# Copyright 2024 Filip Stec
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import socket
from time import sleep, time
import math

from .defines import *

'''
Communication protocol: https://kobuki.readthedocs.io/en/devel/protocol.html
'''

def check_checksum(data):
    chckSum = 0
    for i in range(data[0] + 2):
        chckSum ^= data[i]
    return chckSum  # Returns 0 if everything is correct, otherwise some number

def set_led(led1, led2):
    message = [0xaa, 0x55, 0x04, 0x0c, 0x02, 0x00, (led1 + led2 * 4) % 256, 0x00]
    message[7] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6]
    return message

def set_translation_speed(mmpersec):
    mmpersec = int(mmpersec) & 0xffff
    message = [0xaa, 0x55, 0x0a, 0x0c, 0x02, 0xf0, 0x00, 0x01, 0x04, mmpersec % 256, mmpersec >> 8, 0x00, 0x00, 0x00]
    message[13] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^ message[10] ^ message[11] ^ message[12]
    return message

def set_rotation_speed(radpersec):
    speedvalue = int(radpersec * 230.0 / 2.0) & 0xffff
    message = [0xaa, 0x55, 0x0a, 0x0c, 0x02, 0xf0, 0x00, 0x01, 0x04, speedvalue % 256, speedvalue >> 8, 0x01, 0x00, 0x00]
    message[13] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^ message[10] ^ message[11] ^ message[12]
    return message

def set_arc_speed(mmpersec, radius):
    if math.isclose(radius, 0.0):
        return set_translation_speed(mmpersec)

    speedvalue = int(mmpersec * ((radius + (230 if radius > 0 else -230)) // 2) // radius) & 0xffff
    radius = int(radius) & 0xffff
    message = [0xaa, 0x55, 0x0a, 0x0c, 0x02, 0xf0, 0x00, 0x01, 0x04, speedvalue % 256, speedvalue >> 8, radius % 256, radius >> 8, 0x00]
    message[13] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^ message[10] ^ message[11] ^ message[12]
    return message

def set_sound(noteinHz, duration):
    notevalue = int(1.0 / (noteinHz * 0.00000275) + 0.5)
    message = [0xaa, 0x55, 0x09, 0x0c, 0x02, 0xf0, 0x00, 0x03, 0x03, notevalue % 256, notevalue >> 8, duration % 256, 0x00]
    message[12] = message[2] ^ message[3] ^ message[4] ^ message[5] ^ message[6] ^ message[7] ^ message[8] ^ message[9] ^ message[10] ^ message[11]
    return message

def set_default_pid():
    message = [0xaa, 0x55, 0x13, 0x0c, 0x02, 0xf0, 0x00, 0x0d, 0x0d] + [0x00] * 13
    message[22] = 0
    for i in range(20):
        message[22] ^= message[i + 2]
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
            # timestamp is not published, substitute it with uint16 time in ms
            # output.timestamp = data[checked_value + 1] * 256 + data[checked_value]
            output.timestamp = int(time() * 1e3) & 0xffff
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

def main():
    # Create a UDP socket
    robot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # Message to be sent
        message = set_sound(440, 1000)
        message_bytes = bytes(message)
        # Send the message to the server
        robot_sock.sendto(message_bytes, (UDP_IP, ROBOT_UDP_PORT_UP))

        # Send command
        message = set_rotation_speed(1)
        message_bytes = bytes(message)
        robot_sock.sendto(message_bytes, (UDP_IP, ROBOT_UDP_PORT_UP))

        while True:
            # Receive the response from the server
            # TODO: Implement timeout 
            response, addr = robot_sock.recvfrom(1024)  # Buffer size is 1024 bytes
            kobuki_data = parse_kobuki_message(response)
            print(kobuki_data.EncoderLeft)
            print(kobuki_data.EncoderRight)
            print('-------')
            sleep(0.01)

    # Close the socket
    finally:
        robot_sock.close()


if __name__ == "__main__":
    main()