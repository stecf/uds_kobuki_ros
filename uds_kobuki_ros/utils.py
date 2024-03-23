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


import math

def wrap_angle(angle: float):
    if (angle <= math.pi) and (angle >= -math.pi):
        return angle
    
    if angle < 0.0:
        angle = math.fmod(angle - math.pi, 2.0 * math.pi) - math.pi
    else:
        angle = math.fmod(angle + math.pi, 2.0 * math.pi) - math.pi
    return angle

def calculate_diff_ticks(past_enc, curr_enc):
    diff = curr_enc - past_enc
    half_range = 0xffff / 2

    if abs(diff) > half_range:
        if curr_enc > past_enc:
            return (curr_enc - 0xffff) - past_enc
        else:
            return curr_enc - (past_enc - 0xffff)
    else:
        return diff

def yaw_to_quaternion(yaw):
    # Create a Quaternion message
    quaternion = [None] * 4

    # Convert yaw angle to quaternion
    quaternion[0] = 0.0
    quaternion[1] = 0.0
    quaternion[2] = math.sin(yaw / 2)
    quaternion[3] = math.cos(yaw / 2)

    return quaternion