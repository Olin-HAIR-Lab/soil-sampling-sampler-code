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

t = serial.Serial("/dev/ttyUSB0", 2000000)

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
        0xe8,     # 1  0xc0 Tyep
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
    result = t.write(send_can_id_data)
    time.sleep(0.001)

def can_read(t):
    data = t.read(2)
    hex_data1 = [hex(byte) for byte in data]
    if (data[0] == 0xaa) and (data[1] & 0xc0 == 0xc0):  # frame header
        length = data[1] & 0x0f
        if data[1] & 0x10 == 0x00:
            strFrameFormat = "Data Frame"
        else:
            strFrameFormat = "Remote Frame"

        if data[1] & 0x20 == 0x00:
            strFrameType = "Standard Frame"
            len2 = length + 3
        else:
            strFrameType = "Extended Frame"
            len2 = length + 5

        data2 = t.read(len2)
        hex_data = [hex(byte) for byte in data2]
        hex_data1 += hex_data

        if data2[len2 - 1] == 0x55:  # end code
            if strFrameType == "Standard Frame":
                id = data2[1]
                id <<= 8
                id += data2[0]
                strId = hex(id)

                if length > 0:
                    CanData = hex_data[2:2 + length]
                else:
                    CanData = ["No Data"]
            else:
                id = data2[3]
                id <<= 8
                id += data2[2]
                id <<= 8
                id += data2[1]
                id <<= 8
                id += data2[0]
                strId = hex(id)
                if length > 0:
                    CanData = hex_data[4:4 + length]
                else:
                    CanData = ["No Data"]

            canDataNumeric = [int(x,16) for x in CanData]
            #print(f"Can data numeric: {canDataNumeric}")
            x1 = int.from_bytes(canDataNumeric[0:4],byteorder='little')
            x2 = int.from_bytes(canDataNumeric[4:8],byteorder='little')
            print(f"x1, x2: {x1}, {x2}")




def main(stdscr):
    stdscr.clear()
    stdscr.addstr("Press keys (q to quit)...\n")

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
            signal - MAIN_RETRACT
        elif key == ord('6'):
            signal = PAUSE
        elif key == ord('7'):
            signal = FULL_ROUTINE
        
        if signal > -1:
            print(f"Sent signal {signal}")
            can_send(t,signal)
        stdscr.refresh()
        can_read() # change to actually log data somewhere

curses.wrapper(main)

# def main(count):
#     iterations = 0
#     total_sent = 0
#     total_success = 0
#     while True:

#         # Generate data to send
#         start = random.randint(0,100)
#         data_to_send = [x + start for x in range(count*2)]
#         data_to_send[-1] = 123456789 # use for start command
#         data_to_send[0] = 987654321
#         data_list = data_to_send.copy()
#         data_list.reverse()

#         while len(data_to_send) > 0:
#             x1 = data_to_send.pop()
#             x2 = data_to_send.pop()

#             x1_b = x1.to_bytes(4,'little')
#             x2_b = x2.to_bytes(4,'little')

#             send_can_id_data = bytes([
#                 0xaa,     # 0  Packet header
#                 0xe8,     # 1  0xc0 Tyep
#                 # bit5(frame type 0- standard frame (frame ID 2 bytes), 1-extended frame (frame ID 4 bytes))
#                 # bit4(frame format 0- data frame, 1 remote frame)
#                 # Bit0~3 Frame data length (0~8)
#                 0x67,     # 2  Frame ID data 1    1~8 bit, high bytes at the front, low bytes at the back
#                 0x45,     # 3  Frame ID data 2    1~8 bit, high bytes at the front, low bytes at the back
#                 0x23,     # 4  Frame ID data 3    1~8 bit, high bytes at the front, low bytes at the back
#                 0x01,     # 5  Frame ID data 4    9~16 bit, high bytes at the front, low bytes at the back
#                 x1_b[0],     # 6  Frame data 1       CAN sends  data 1
#                 x1_b[1],     # 7  Frame data 2       CAN sends  data 2
#                 x1_b[2],     # 8  Frame data 3       CAN sends  data 3
#                 x1_b[3],     # 9  Frame data 4       CAN sends  data 4
#                 x2_b[0],     # 10 Frame data 5       CAN sends  data 5
#                 x2_b[1],     # 11 Frame data 6       CAN sends  data 6
#                 x2_b[2],     # 12 Frame data 7       CAN sends  data 7
#                 x2_b[3],     # 13 Frame data 8       CAN sends  data 8
#                 0x55,     # 14 This guy needs to be 0x55
#             ])
#             num = t.write(send_can_id_data)
#             #print(f"Ints: {x1, x2}")
#             time.sleep(0.001) # data goes bad if we have no sleep

#         received_count = 0
#         nums_received = []
#         while received_count < count*2:
#             data = t.read(2)
#             hex_data1 = [hex(byte) for byte in data]
#             if (data[0] == 0xaa) and (data[1] & 0xc0 == 0xc0):  # frame header
#                 length = data[1] & 0x0f
#                 if data[1] & 0x10 == 0x00:
#                     strFrameFormat = "Data Frame"
#                 else:
#                     strFrameFormat = "Remote Frame"

#                 if data[1] & 0x20 == 0x00:
#                     strFrameType = "Standard Frame"
#                     len2 = length + 3
#                 else:
#                     strFrameType = "Extended Frame"
#                     len2 = length + 5

#                 data2 = t.read(len2)
#                 hex_data = [hex(byte) for byte in data2]
#                 hex_data1 += hex_data
#                 #print(hex_data1)
#                 if data2[len2 - 1] == 0x55:  # end code
#                     if strFrameType == "Standard Frame":
#                         id = data2[1]
#                         id <<= 8
#                         id += data2[0]
#                         strId = hex(id)

#                         if length > 0:
#                             CanData = hex_data[2:2 + length]
#                         else:
#                             CanData = ["No Data"]
#                     else:
#                         id = data2[3]
#                         id <<= 8
#                         id += data2[2]
#                         id <<= 8
#                         id += data2[1]
#                         id <<= 8
#                         id += data2[0]
#                         strId = hex(id)
#                         if length > 0:
#                             CanData = hex_data[4:4 + length]
#                         else:
#                             CanData = ["No Data"]

#                     canDataNumeric = [int(x,16) for x in CanData]
#                     #print(f"Can data numeric: {canDataNumeric}")
#                     x1 = int.from_bytes(canDataNumeric[0:4],byteorder='little')
#                     x2 = int.from_bytes(canDataNumeric[4:8],byteorder='little')
#                     received_count = received_count + 2
#                     nums_received.append(x1)
#                     nums_received.append(x2)
#                     #print("Receive CAN id: " + strId + " Data:", end='')
#                     #print(f" {x1}, {x2}")
#                     #print(strFrameType + ", " + strFrameFormat)
#                 else:
#                     print("Receive Packet header Error")
            
#         # Now we've received our data back
#         print(data_list)
#         print(nums_received)
#         if (len(data_list) != len(nums_received)):
#             print("Wrong number of numbers received")
#         else:
#             success_list = [1 for i in range(len(data_list)) if data_list[i] == nums_received[i]]
#             successes = sum(success_list)
#             total_success += successes
#             total_sent += len(data_list)
#             print(f"Received {successes * 4} bytes correct out of {len(data_list) * 4} bytes for a success rate of {successes / len(data_list)}")
#             print(f"\nTotal: {total_sent * 4} bytes sent, {100 * total_success / total_sent}% returned successfully. \n")
        
#         iterations += 1
#         time.sleep(1)

#     t.close()

# if __name__ == "__main__":
#     if len(sys.argv) > 1:
#         count = sys.argv[1]
#         main(int(count))
#     else:
#         main(16)