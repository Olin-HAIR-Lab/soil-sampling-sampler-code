import serial
import time
import sys
import random
import curses

START = 0
STOP = 1
CLAW_EXTEND = 2
CLAW_RETRACT = 3
MAIN_EXTEND = 4
MAIN_RETRACT = 5
PAUSE = 6
FULL_ROUTINE = 7

START_SIGNAL = 123456789
END_SIGNAL = 987654321

t = serial.Serial("/dev/ttyUSB0", 2000000, timeout=0.05)

print(t.portstr)

def calculate_checksum(data):
    checksum = sum(data[2:])
    return checksum & 0xff

def can_setup(t):
    set_can_baudrate = [
        0xaa,     #  0  Packet header
        0x55,     #  1  Packet header
        0x12,     #  3 Type: use variable protocol to send and receive data##  0x02- Setting (using fixed 20 byte protocol to send and receive data),   0x12- Setting (using variable protocol to send and receive data)##
        0x03,     #  3 CAN Baud Rate:  500kbps  ##  0x01(1Mbps),  0x02(800kbps),  0x03(500kbps),  0x04(400kbps),  0x05(250kbps),  0x06(200kbps),  0x07(125kbps),  0x08(100kbps),  0x09(50kbps),  0x0a(20kbps),  0x0b(10kbps),   0x0c(5kbps)##
        0x01,     #  4  Frame Type: Extended Frame  ##   0x01 standard frame,   0x02 extended frame ##
        0x00,     #  5  Filter ID1
        0x00,     #  6  Filter ID2
        0x00,     #  7  Filter ID3
        0x00,     #  8  Filter ID4
        0x00,     #  9  Mask ID1
        0x00,     #  10 Mask ID2
        0x00,     #  11 Mask ID3
        0x00,     #  12 Mask ID4
        0x00,     #  13 CAN mode:  normal mode  ##   0x00 normal mode,   0x01 silent mode,   0x02 loopback mode,   0x03 loopback silent mode ##
        0x00,     #  14 automatic resend:  automatic retransmission
        0x00,     #  15 Spare
        0x00,     #  16 Spare
        0x00,     #  17 Spare
        0x00,     #  18 Spare
    ]

    # Calculate checksum
    checksum = calculate_checksum(set_can_baudrate)
    set_can_baudrate.append(checksum)
    #print(set_can_baudrate)
    set_can_baudrate = bytes(set_can_baudrate)

    # Send command to set CAN baud rate
    num_set_baud = t.write(set_can_baudrate)
    print(f"Set CAN baud rate command sent, bytes written: {num_set_baud}")
    time.sleep(0.5)

def can_send(t,signal):
    # signal is an int that we want to turn into bytes
    signal_b = signal.to_bytes(4,'little')
    send_can_id_data = bytes([
        0xaa,     # 0  Packet header
        0xc8,     # 1  0xc0 Tyep
        # bit5(frame type 0- standard frame (frame ID 2 bytes), 1-extended frame (frame ID 4 bytes))
        # bit4(frame format 0- data frame, 1 remote frame)
        # Bit0~3 Frame data length (0~8)
        0x67,     # 2  Frame ID data 1    1~8 bit, high bytes at the front, low bytes at the back
        0x45,     # 3  Frame ID data 2    1~8 bit, high bytes at the front, low bytes at the back
        0x23,     # 4  Frame ID data 3    1~8 bit, high bytes at the front, low bytes at the back
        0x01,     # 5  Frame ID data 4    9~16 bit, high bytes at the front, low bytes at the back
        signal_b[0],     # 6  Frame data 1       CAN sends  data 1
        signal_b[1],     # 7  Frame data 2       CAN sends  data 2
        signal_b[2],     # 8  Frame data 3       CAN sends  data 3
        signal_b[3],     # 9  Frame data 4       CAN sends  data 4
        0x00,     # 10 Frame data 5       CAN sends  data 5
        0x00,     # 11 Frame data 6       CAN sends  data 6
        0x00,     # 12 Frame data 7       CAN sends  data 7
        0x00,     # 13 Frame data 8       CAN sends  data 8
        0x55,     # 14 This guy needs to be 0x55
    ])

    send_can_id_data = bytes([
        0xaa,
        0xc8,       # standard data frame, DLC=8
        0x12,       # standard ID low byte
        0x34,       # standard ID high byte
        signal_b[0],
        signal_b[1],
        signal_b[2],
        signal_b[3],
        0x00,
        0x00,
        0x00,
        0x00,
        0x55,
    ])
    result = t.write(send_can_id_data)
    time.sleep(0.001)

def dump_serial_bytes(t, max_bytes=64):
    n = t.in_waiting
    if n <= 0:
        return None
    data = t.read(min(n, max_bytes))
    return " ".join(f"{b:02X}" for b in data)

def can_read(t):
    data = t.read(2)
    if len(data) < 2:
        return None

    if (data[0] == 0xaa) and (data[1] & 0xc0 == 0xc0):
        length = data[1] & 0x0f

        if data[1] & 0x20 == 0x00:
            # standard frame
            len2 = length + 3
            data2 = t.read(len2)
            if len(data2) < len2:
                return None

            if data2[-1] != 0x55:
                return None

            can_data = list(data2[2:2 + length])
        else:
            len2 = length + 5
            data2 = t.read(len2)
            if len(data2) < len2:
                return None

            if data2[-1] != 0x55:
                return None

            can_data = list(data2[4:4 + length])

        if len(can_data) >= 8:
            x1 = int.from_bytes(can_data[0:4], byteorder='little', signed=True)
            x2 = int.from_bytes(can_data[4:8], byteorder='little', signed=True)
            return (x1, x2)

    return None


def main(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)

    received_data = []
    receiving = False

    stdscr.clear()
    stdscr.addstr("Press keys (q to quit)...\n")
    can_setup(t)

    last_raw = "No data yet"
    last_status = "Press 0-7, q to quit"

    while True:
        key = stdscr.getch()
        signal = -1
        if key == ord('0'):
            signal = START
        elif key == ord('1'):
            signal = STOP
        elif key == ord('2'):
            signal = CLAW_EXTEND
        elif key == ord('3'):
            signal = CLAW_RETRACT
        elif key == ord('4'):
            signal = MAIN_EXTEND
        elif key == ord('5'):
            signal = MAIN_RETRACT
        elif key == ord('6'):
            signal = PAUSE
        elif key == ord('7'):
            signal = FULL_ROUTINE
        
        if signal > -1:
            last_status = f"Sent signal {signal}"
            can_send(t,signal)
        
        raw = dump_serial_bytes(t)
        if raw:
            last_raw = raw

        stdscr.erase()
        stdscr.addstr(0, 0, last_status)
        stdscr.addstr(2, 0, f"in_waiting: {t.in_waiting}")
        stdscr.addstr(4, 0, "raw bytes received:")
        stdscr.addstr(5, 0, last_raw[:120])
        stdscr.refresh()

        time.sleep(0.02)

        # result = can_read(t)

        # if result:
        #     x1, x2 = result

        #     if x1 == START_SIGNAL:
        #         receiving = True
        #         received_data = []
        #     elif x1 == END_SIGNAL:
        #         receiving = False
        #     elif receiving:
        #         received_data.append(x1)
        #         if x2 != 0:
        #             received_data.append(x2)

        # # --- display ---
        # stdscr.clear()
        # stdscr.addstr(0, 0, "Receiving CAN data:")

        # for i, val in enumerate(received_data[-10:]):  # show last 10
        #     stdscr.addstr(2 + i, 0, str(val))

        # stdscr.refresh()
        # time.sleep(0.01)

curses.wrapper(main)
