#include <Servo.h>
Servo s;

void setup() {
  Serial.begin(115200);
  s.attach(11);
}

void loop() {
  Serial.println("stop");
  s.write(90);
  delay(1000);
  s.writeMicroseconds(1500);
  delay(1000);

  Serial.println("forward");
  s.write(180);
  delay(2000);
  s.writeMicroseconds(2100);
  delay(3000);

  Serial.println("stop");
  s.write(90);
  delay(1000);
  s.writeMicroseconds(1500);
  delay(2000);

  Serial.println("reverse");
  s.write(0);
  delay(2000);
  s.writeMicroseconds(900);
  delay(3000);
}