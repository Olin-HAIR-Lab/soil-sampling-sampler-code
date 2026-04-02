#include <mcp_can.h>
#include <SPI.h>
#include <stdint.h>
#include <Servo.h>

/***************************
    Constants to adjust
****************************/

// Pins for servos, sensor, and sampler
const uint8_t SERVO1_PIN = 8;
const uint8_t SERVO2_PIN = 9;
const uint8_t SERVO3_PIN = 10;
const uint8_t CONTINUOUS_PIN = 11;
const uint8_t BUS_SERVO_PIN = 5;
const uint8_t SENSOR_PIN = A0; // moisture sensor
const uint8_t DISTANCE_PIN = A1; // distance sensor

// Pins for CAN communication
int CAN0_INT = 2;
MCP_CAN CAN0(10);

// Movement constants --- how far to require the auger to extend and contract
const float MINIMUM_EXTENSION_DIST = 80.0; // mm from fully retracted, may need to change units
const float MAXIMUM_RETRACTION_DIST = 5.0; // mm from fully retracted

// Timing constants --- how long, in ms, different tasks can take
const int EXTENSION_TIME_LIMIT = 20 * 1000; // ms
const int RETRACTION_TIME_LIMIT = 20 * 1000;
const int CLAW_EXTENSION_TIME_LIMIT = 2 * 1000;
const int CLAW_RETRACTION_TIME_LIMIT = 2 * 1000;
const int SAMPLING_TIME = 2 * 1000;
const int WAIT_AFTER_SEND_TIME = 500;
const int CAN_SEND_DELAY = 10;
const int SAMPLING_DELAY = 50;

// Movement controls --- angle of claws, speed of claws (deg/s), speed percent of auger
const float CLAW_ANGLE_START = 60.0;
const float CLAW_ANGLE_END = 90.0;
const float CLAW_SPEED = 60.0;
const int AUGER_SPEED_PERCENT = 50;

// Sentinel values to send to the Python code running on the drone
const int32_t START_SIGNAL = 123456789;
const int32_t END_SIGNAL = 987654321;
const int32_t FAIL_SIGNAL = -1;

///////////////////////////////////////////////////////////////////////////

struct ManagedServo {
  Servo hw;

  uint8_t pin;
  bool attached = false;
  bool enabled = false;
  bool inverted = false;

  float currentAngle = 90.0f;   // logical angle
  float targetAngle  = 90.0f;   // logical angle
  float speedDegPerSec = 60.0f;

  unsigned long lastUpdateMs = 0;

  int neutralWrite = 90;        // for continuous servo neutral position
};

struct ManagedContinuousServo {
  Servo hw;
  uint8_t pin;

  bool attached = false;
  bool enabled = false;
  bool inverted = false;

  int commandPct = 0;  // direction, -100 to 100
  int neutralUs = 1500; // microseconds of neutral (no rotation)
  int maxForwardUs = 1700; // max rotation speed forwards, in microseconds
  int maxReverseUs = 1300; // max rotation speed backwards, in microseconds
};

enum CANCommand {
  START = 0,          // begin listening to other commands
  STOP = 1,           // stop all movement until START
  CLAW_EXTEND = 2,    // drive smaller servos
  CLAW_RETRACT = 3,   // drive smaller servos backward
  MAIN_EXTEND = 4,    // drive larger servo
  MAIN_RETRACT = 5,   // drive larger servo backward
  PAUSE = 6,          // halt movement
  FULL_ROUTINE = 7    // do the automated sampling routine
};

enum SamplerMode {
  SAMPLER_OFF = 0,
  SAMPLER_PAUSED = 1,
  SAMPLER_CLAW_EXTENDING = 2,
  SAMPLER_CLAW_RETRACTING = 3,
  SAMPLER_EXTENDING = 4,
  SAMPLER_RETRACTING = 5,
  SAMPLER_CHECK_EXTENSION = 6,
  SAMPLER_CHECK_RETRACTION = 7,
  SAMPLER_SAMPLING = 8,
  SAMPLER_WAIT_AFTER_SEND = 9
};



long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

SamplerMode state = SAMPLER_OFF;
unsigned long state_end_ms; // when will the next state end?  
int readings[128]; // hold the raw values from the moisture sensor pin
int reading_count = 0; // how many readings have we read

float extension;
int succeeded;
int start_delay_ms;

ManagedServo servo1;
ManagedServo servo2;
ManagedServo servo3;
ManagedContinuousServo auger_servo;
Servo busServo;

void initArmServos() {
  initManagedServo(servo1, SERVO1_PIN, true,  0.0f, 60.0f);   // inverted
  initManagedServo(servo2, SERVO2_PIN, false, 0.0f, 60.0f);
  initManagedServo(servo3, SERVO3_PIN, true,  0.0f, 60.0f);   // inverted

  enableServo(servo1);
  enableServo(servo2);
  enableServo(servo3);
}

void updateAllServos() {
  unsigned long now = millis();
  updateManagedServo(servo1, now);
  updateManagedServo(servo2, now);
  updateManagedServo(servo3, now);
}

void initManagedServo(ManagedServo& s, uint8_t pin, bool inverted, float initialAngle, float speedDegPerSec) {
  s.pin = pin;
  s.inverted = inverted;
  s.currentAngle = clampAngle(initialAngle);
  s.targetAngle = s.currentAngle;
  s.speedDegPerSec = speedDegPerSec;
  s.lastUpdateMs = millis();
  s.enabled = false;
  s.attached = false;
}

void enableServo(ManagedServo& s) {
  if (!s.attached) {
    s.hw.attach(s.pin);
    s.attached = true;
  }
  s.enabled = true;
  s.hw.write(logicalToPhysicalWrite(s, s.currentAngle));
}

void disableServo(ManagedServo& s) {
  s.enabled = false;

  if (s.attached) {
    s.hw.detach();
    s.attached = false;
  }
}

void setServoTarget(ManagedServo& s, float targetAngle) {
  s.targetAngle = clampAngle(targetAngle);
}

void setServoSpeed(ManagedServo& s, float speedDegPerSec) {
  if (speedDegPerSec < 0.0f) speedDegPerSec = 0.0f;
  s.speedDegPerSec = speedDegPerSec;
}

void setServoInverted(ManagedServo& s, bool inverted) {
  s.inverted = inverted;
  if (s.enabled && s.attached) {
    s.hw.write(logicalToPhysicalWrite(s, s.currentAngle));
  }
}

void updateManagedServo(ManagedServo& s, unsigned long now_ms) {
  if (!s.enabled || !s.attached) {
    s.lastUpdateMs = now_ms;
    return;
  }

  unsigned long dtMs = now_ms - s.lastUpdateMs;
  if (dtMs == 0) {
    return;
  }

  s.lastUpdateMs = now_ms;

  float maxStep = s.speedDegPerSec * ((float)dtMs / 1000.0f);
  float error = s.targetAngle - s.currentAngle;

  if (error > maxStep) {
    s.currentAngle += maxStep;
  } else if (error < -maxStep) {
    s.currentAngle -= maxStep;
  } else {
    s.currentAngle = s.targetAngle;
  }

  s.currentAngle = clampAngle(s.currentAngle);
  s.hw.write(logicalToPhysicalWrite(s, s.currentAngle));
}

void updateContinuousServo(ManagedContinuousServo& s) {
  if (!s.enabled || !s.attached) return;

  int cmd = s.commandPct;
  if (s.inverted) cmd = -cmd;

  int pulse = s.neutralUs;
  // Passed value is neutral + (input / 100) * (forward - neutral)
  if (cmd > 0) {
    pulse = s.neutralUs + (cmd * (s.maxForwardUs - s.neutralUs)) / 100;
  } else if (cmd < 0) {
    pulse = s.neutralUs - ((-cmd) * (s.neutralUs - s.maxReverseUs)) / 100;
  }

  s.hw.writeMicroseconds(pulse);
}

void setBusServoNeutral() {
  busServo.write(90);
}

void setBusServoRotate() {
  busServo.write(180);
}

void rotateBusServoForMs(unsigned long durationMs) {
  setBusServoRotate();
  delay(durationMs);
  setBusServoNeutral();
}

void i32_to_bytes(unsigned char* dst, const uint32_t x) {
  dst[0] = (unsigned char)(x & 0xFF);
  dst[1] = (unsigned char)((x >> 8) & 0xFF);
  dst[2] = (unsigned char)((x >> 16) & 0xFF);
  dst[3] = (unsigned char)((x >> 24) & 0xFF);
}

int32_t read_i32_le(const uint8_t* p) {
  return (int32_t)(
      ((uint32_t)p[0]) |
      ((uint32_t)p[1] << 8) |
      ((uint32_t)p[2] << 16) |
      ((uint32_t)p[3] << 24)
  );
}

float clampAngle(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 180.0f) return 180.0f;
  return x;
}

int logicalToPhysicalWrite(const ManagedServo& s, float logicalAngle) {
  logicalAngle = clampAngle(logicalAngle);

  if (s.inverted) {
    logicalAngle = 180.0f - logicalAngle;
  }

  return (int)(logicalAngle + 0.5f);
}

void processCANReceived() {
  if(CAN0.checkReceive() == CAN_MSGAVAIL)  {
    if (CAN0.readMsgBuf(&rxId, &len, rxBuf) != CAN_OK) {
      return;
    }
      
    bool isExtended = (rxId & 0x80000000UL) != 0;
    bool isRemote   = (rxId & 0x40000000UL) != 0;
    uint32_t canId  = isExtended ? (rxId & 0x1FFFFFFFUL) : rxId;
    
    if (isExtended) { // we aren't expecting extended CAN frames
      return;
    }

    if (isRemote) { // we aren't expecting to use remote frames
      return;
    }

    if (len < 4) {
      // We don't have a full int of data, so we throw it out
      return;
    }

    int32_t cmd = read_i32_le(&rxBuf[0]); // read from char buffer, interpret as a 4 byte int
    Serial.print("Received command: ");
    Serial.println(cmd);
    if (cmd > 7) {
      Serial.println("Received a command greater than 7, which shouldn't be supported");
    } else {
      processCommand(cmd);
    }
  }
}

void commandServo(ManagedServo& s, float targetAngle, float speedDegPerSec) {
  s.targetAngle = clampAngle(targetAngle);
  if (speedDegPerSec < 0.0f) speedDegPerSec = 0.0f;
  s.speedDegPerSec = speedDegPerSec;
}

void commandContinuousServo(ManagedContinuousServo& s, int commandPct) {
  if (commandPct > 100) commandPct = 100;
  if (commandPct < -100) commandPct = -100;
  s.commandPct = commandPct;
}

void setClawTargets(float logicalAngle, float speedDegPerSec) {
  // set target angles and speeds for the three claw servos
  commandServo(servo1, logicalAngle, speedDegPerSec);
  commandServo(servo2, logicalAngle, speedDegPerSec);
  commandServo(servo3, logicalAngle, speedDegPerSec);
}

int stateTimedOut(unsigned long time_now) {
  return (time_now >= state_end_ms);
}

float getExtensionDistance() {
  // Read the distance from the distance sensor
  // Use distance at max retraction to calculate how far we've extended
  return 0.0; //TODO
}

void logSensorData(int reading_count) {
  readings[reading_count] = analogRead(SENSOR_PIN); // read the pin voltage
}

void sendTwoInts(int32_t a, int32_t b) {
  // Pack two ints in the 8 bytes of data in a CAN frame and send over CAN
  uint8_t buf[8];

  i32_to_bytes(buf, a);
  i32_to_bytes(buf + 4, b);

  byte sndStat = CAN0.sendMsgBuf(0x100, 0, 8, buf);

  Serial.print("Sent: ");
  for (int k = 0; k < 8; k++) {
    if (buf[k] < 16) Serial.print('0');
    Serial.print(buf[k], HEX);
    Serial.print(' ');
  }
  Serial.println();
  delay(5);
}

void sendIntArrayWithSentinel(const int* data, int count) {
  // Send an array of ints over CAN in pairs, appending a sentinel to the start and end
  sendTwoInts(START_SIGNAL, 0); // START_SIGNAL value indicates success + data being sent

  for (uint16_t i = 0; i < count; i += 2) {
    int32_t a = data[i]; // cast to fixed size int for consistency when sending
    int32_t b = 0;

    if ((i + 1) < count) {
      b = data[i + 1];
    }

    sendTwoInts(a, b);
  }
  sendTwoInts(END_SIGNAL, 0); // END_SIGNAL indicates we're done, drone can take off now
}

void sendFailure() {
  // We failed something in the sampling process, send -1 to signify this 
  sendTwoInts(FAIL_SIGNAL, FAIL_SIGNAL);
}

void processCommand(int cmd) {
  // Switch statement responding to a command sent over CAN

  switch(cmd) {
    case START:
      state = SAMPLER_PAUSED;
      Serial.println("Entering SAMPLER_PAUSED state");
      break;
    
    case STOP:
      state = SAMPLER_OFF;
      Serial.println("Entering SAMPLER_OFF state");
      setClawTargets(CLAW_ANGLE_START, 0.0); // speed at 0.0 makes the claws stop moving
      commandContinuousServo(auger_servo, 0); // speed 0 makes the auger stop moving
      break;

    case CLAW_EXTEND:
      setClawTargets(CLAW_ANGLE_END, CLAW_SPEED);
      break;

    case CLAW_RETRACT:
      setClawTargets(CLAW_ANGLE_START, CLAW_SPEED);
      break;
    
    case MAIN_EXTEND:
      commandContinuousServo(auger_servo, AUGER_SPEED_PERCENT);
      break;

    case MAIN_RETRACT:
      commandContinuousServo(auger_servo, -1*AUGER_SPEED_PERCENT);
      break;

    case PAUSE:
      setClawTargets(CLAW_ANGLE_START, 0.0); // speed at 0.0 makes the claws stop moving
      commandContinuousServo(auger_servo, 0); // speed 0 makes the auger stop moving
      break;

    case FULL_ROUTINE:
      state = SAMPLER_CLAW_EXTENDING;
      Serial.println("Entering SAMPLER_CLAW_EXTENDING state");
      state_end_ms = millis() + CLAW_EXTENSION_TIME_LIMIT;
      break;
  }
}

void loop() {
  processCANReceived(); // listen for any new instructions
  switch(state) {
    case SAMPLER_OFF:
      // We haven't yet received a start command
      // Make sure servos are not moving
      break;

    case SAMPLER_CLAW_EXTENDING:
      // We should drive the servos on the claws
      if (stateTimedOut(millis())) {
        state = SAMPLER_EXTENDING;
        Serial.println("Entering SAMPLER_EXTENDING state");
        state_end_ms = millis() + EXTENSION_TIME_LIMIT;
      } else {
        // keep claw servos extending
        setClawTargets(CLAW_ANGLE_END, CLAW_SPEED); // Target angle of extension, deg/s
      }
      break;

    case SAMPLER_EXTENDING:
      extension = getExtensionDistance(); // how far have we gone down?
      commandContinuousServo(auger_servo, AUGER_SPEED_PERCENT);
      if (extension > MINIMUM_EXTENSION_DIST) {
        // we have extended enough
        state = SAMPLER_SAMPLING;
        Serial.println("Entering SAMPLER_SAMPLING state");
        start_delay_ms = millis();
        state_end_ms = millis() + SAMPLING_TIME;
        break;
      }
      if (stateTimedOut(millis())) {
        // We ran out of time --- unable to extend for some reason
        Serial.println("Failed to extend in given time");
        state = SAMPLER_RETRACTING;
        Serial.println("Entering SAMPLER_RETRACTING state");
        state_end_ms = millis() + RETRACTION_TIME_LIMIT;
        succeeded = 0;
      }
      break;

    case SAMPLER_SAMPLING:
      // We're holding position and sampling the soil
      if (stateTimedOut(millis())) {
        // We've sampled for enough time (success)
        state = SAMPLER_RETRACTING;
        Serial.println("Entering SAMPLER_RETRACTING state");
        succeeded = 1;
        state_end_ms = millis() + RETRACTION_TIME_LIMIT;
      } else {
        if (millis() > start_delay_ms + SAMPLING_DELAY) {
          logSensorData(reading_count);
          start_delay_ms = millis(); // record the time so we can delay the next sample
          reading_count++;
          if (reading_count > 128) {
            // We're out of space to store values
            Serial.println("Filled the array of readings");
            state = SAMPLER_RETRACTING;
            succeeded = 1;
            state_end_ms = millis() + RETRACTION_TIME_LIMIT;
          }
        }
      }
      break;

    case SAMPLER_RETRACTING:
      extension = getExtensionDistance(); // how far have we gone down?
      commandContinuousServo(auger_servo, -1*AUGER_SPEED_PERCENT);
      if (extension < MAXIMUM_RETRACTION_DIST) {
        // we have retracted enough
        // We still need to undo the claws
        state = SAMPLER_CLAW_RETRACTING;
        Serial.println("Entering SAMPLER_CLAW_RETRACTING state");
        state_end_ms = millis() + RETRACTION_TIME_LIMIT;
        break;
      }
      if (stateTimedOut(millis())) {
        // We ran out of time --- unable to retract for some reason
        Serial.println("Failed to retract auger in given time");

        // We still need to undo the claws
        state = SAMPLER_CLAW_RETRACTING;
        Serial.println("Entering SAMPLER_CLAW_RETRACTING state");
        state_end_ms = millis() + RETRACTION_TIME_LIMIT;
      }
      break;

    case SAMPLER_CLAW_RETRACTING:
      // We move the smaller servos to retract the claws
      setClawTargets(CLAW_ANGLE_START, CLAW_SPEED); // Target angle of retraction, deg/s
      if (stateTimedOut(millis())) {
        // did we succeed? Only send data if so
        if (succeeded) {
          sendIntArrayWithSentinel(readings,reading_count); // send the data back over CAN
          state_end_ms = millis() + WAIT_AFTER_SEND_TIME;
          state = SAMPLER_WAIT_AFTER_SEND;
          Serial.println("Entering SAMPLER_WAIT_AFTER_SEND state");
          break;

        } else {
          sendFailure(); // tell the drone we couldn't get data
          state_end_ms = millis() + WAIT_AFTER_SEND_TIME;
          state = SAMPLER_WAIT_AFTER_SEND;
          Serial.println("Entering SAMPLER_WAIT_AFTER_SEND state");
          break;
        }
      }
      break;
    
    case SAMPLER_WAIT_AFTER_SEND:
      if (stateTimedOut(millis())) {
        // We've allotted enough time for the drone to receive the data
        // Reset any variables we need to
        reading_count = 0;
        Serial.println("Entering SAMPLER_OFF state");
        state = SAMPLER_OFF;
      }
      break;

    case SAMPLER_PAUSED:
    default:
      // Waiting for manual control --- don't do anything here
      break;

  }
  updateAllServos();
  updateContinuousServo(auger_servo);
}

void setup()
{
  Serial.begin(115200);
  
  // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
}