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
import struct

from .defines import *

def parse_lidar_message(data):
    result = TLaserMeasurement()
    result.numberOfScans = len(data) // 24

    for i in range(result.numberOfScans):
        offset = i * 24
        laser_data = LaserData()
        # scanQuality is of type int, which is 4 bytes padded by 4 bytes Big Endian
        laser_data.scanQuality = int.from_bytes(data[offset + 4:offset + 8], byteorder='big')
        # scanAngle is of type double, which is 8 bytes Big Endian
        laser_data.scanAngle = struct.unpack('<d', data[offset + 8:offset + 16])[0]
        # scanDistance is of type double, which is 8 bytes Big Endian
        laser_data.scanDistance = struct.unpack('<d', data[offset + 16:offset + 24])[0]
        result.Data.append(laser_data)
    
    return result

def main():
    # Create a UDP socket
    lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # Message to be sent
        message = 0x00
        message_bytes = bytes(message)
        # Send the message to the server
        lidar_sock.sendto(message_bytes, (UDP_IP, LIDAR_UDP_PORT_UP))

        while True:
            # Receive the response from the server
            # TODO: Implement timeout 
            response, _ = lidar_sock.recvfrom(1000*24)  # Buffer size is 1000 * LaserData (24 bytes)
            lidar_data = parse_lidar_message(response)

            print("-" * 20)
            print("Number of scans", lidar_data.numberOfScans)
            print(0, lidar_data.Data[0])
            print(1, lidar_data.Data[1])
            print(len(lidar_data.Data) - 1, lidar_data.Data[-1])

    # Close the socket
    finally:
        lidar_sock.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass