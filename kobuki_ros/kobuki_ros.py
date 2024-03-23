import rclpy
from rclpy.node import Node

import socket
import threading
from time import sleep

from .defines import *

def check_checksum(data):
    chckSum = 0
    for i in range(data[0] + 2):
        chckSum ^= data[i]
    return chckSum  # Returns 0 if everything is correct, otherwise some number

def set_sound(noteinHz, duration):
    notevalue = int(1.0 / (noteinHz * 0.00000275) + 0.5)
    message = [0xaa, 0x55, 0x09, 0x0c, 0x02, 0xf0, 0x00, 0x03, 0x03, notevalue % 256, notevalue >> 8, duration % 256, 0x00]
    message[12] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^ message[10] ^ message[11]
    return message

def set_rotation_speed(radpersec):
    speedvalue = int(radpersec * 230.0 / 2.0)
    message = [0xaa, 0x55, 0x0a, 0x0c, 0x02, 0xf0, 0x00, 0x01, 0x04, speedvalue % 256, speedvalue >> 8, 0x01, 0x00, 0x00]
    message[13] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^ message[10] ^ message[11] ^ message[12]
    return message

def parse_kobuki_message(data):
    return_value = check_checksum(data)
    # if checksum is bad, then ignore
    if return_value != 0:
        Warning('Bad checksum ignoring message')

    output = TKobukiData()

    checked_value = 1
    # while not at the end of the data length
    while checked_value < data[0]:
        # basic data subload
        if data[checked_value] == 0x01:
            checked_value += 1
            if data[checked_value] != 0x0F:
                Warning('Bad ts byte ignoring message')
            checked_value += 1
            output.timestamp = data[checked_value + 1] * 256 + data[checked_value]
            checked_value += 2
            output.BumperCenter = bool(data[checked_value] & 0x02)
            output.BumperLeft = bool(data[checked_value] & 0x04)
            output.BumperRight = bool(data[checked_value] & 0x01)
            checked_value += 1
            output.WheelDropLeft = bool(data[checked_value] & 0x02)
            output.WheelDropRight = bool(data[checked_value] & 0x01)
            checked_value += 1
            output.CliffCenter = bool(data[checked_value] & 0x02)
            output.CliffLeft = bool(data[checked_value] & 0x04)
            output.CliffRight = bool(data[checked_value] & 0x01)
            checked_value += 1
            output.EncoderLeft = data[checked_value + 1] * 256 + data[checked_value]
            checked_value += 2
            output.EncoderRight = data[checked_value + 1] * 256 + data[checked_value]
            checked_value += 2
            output.PWMleft = data[checked_value]
            checked_value += 1
            output.PWMright = data[checked_value]
            checked_value += 1
            output.ButtonPress = data[checked_value]
            checked_value += 1
            output.Charger = data[checked_value]
            checked_value += 1
            output.Battery = data[checked_value]
            checked_value += 1
            output.overCurrent = data[checked_value]
            checked_value += 1
        elif data[checked_value] == 0x03:
            checked_value += 1
            if data[checked_value] != 0x03:
                return -3
            checked_value += 1
            output.IRSensorRight = data[checked_value]
            checked_value += 1
            output.IRSensorCenter = data[checked_value]
            checked_value += 1
            output.IRSensorLeft = data[checked_value]
            checked_value += 1
        # other conditions follow the same pattern...
        # omitted for brevity

        else:
            checked_value += 1
            checked_value += data[checked_value] + 1

    return output

class Kobuki(Node):
    stop_flag = False

    def __init__(self):
        super().__init__('kobuki')

        self.port = ROBOT_UDP_PORT
        self.robot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Message to be sent
        message = set_sound(440, 1000)
        message_bytes = bytes(message)
        # Send the message to the server
        self.robot_sock.sendto(message_bytes, (UDP_IP, ROBOT_UDP_PORT))

        # Send command
        message = set_rotation_speed(1)
        message_bytes = bytes(message)
        self.robot_sock.sendto(message_bytes, (UDP_IP, ROBOT_UDP_PORT))

        self.receiver_thread = threading.Thread(target=self.udp_receiver)
        self.receiver_thread.start()

    def __del__(self):
        self.robot_sock.close()

    def udp_receiver(self):
        while not self.stop_flag:
            response, addr = self.robot_sock.recvfrom(1024)
            kobuki_data = parse_kobuki_message(response)
            print(kobuki_data.EncoderLeft)
            print(kobuki_data.EncoderRight)
            print('-------')
            sleep(0.01)
        print("While loop ended")

def main(args=None):
    rclpy.init(args=args)

    kobuki = Kobuki()

    rclpy.spin(kobuki)

    # Destroy the node explicitly
    kobuki.destroy_node()
    del kobuki
    rclpy.shutdown()

if __name__ == '__main__':
    main()