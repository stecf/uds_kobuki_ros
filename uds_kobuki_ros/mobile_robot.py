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


from .utils import *

class MobileRobot:
    # robot parameters
    bias = 0.23 # wheelbase, wheel_to_wheel, in [m]
    wheel_radius = 0.035 # radius of main wheel, in [m]
    tick_to_rad = 0.002436916871363930187454

    # diff drive
    init_diff_drive = False
    last_timestamp = 0
    last_diff_time = 0.0
    last_velocity_left = 0.0
    last_velocity_right = 0.0
    last_tick_left = 0
    last_tick_right = 0
    last_rad_left = 0.0
    last_rad_right = 0.0

    def __init__(self):
        pass

    def update_diff_drive(self, timestamp, left_encoder, right_encoder):
        if timestamp == 0:
            self.last_timestamp = timestamp
            self.last_tick_left = left_encoder
            self.last_tick_right = right_encoder
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        
        left_diff_ticks = calculate_diff_ticks(self.last_tick_left, left_encoder)
        self.last_tick_left = left_encoder
        self.last_rad_left += self.tick_to_rad * left_diff_ticks

        right_diff_ticks = calculate_diff_ticks(self.last_tick_right, right_encoder)
        self.last_tick_right = right_encoder
        self.last_rad_right += self.tick_to_rad * right_diff_ticks

        pose_update = self.pose_update_from_wheel_differential(
            self.tick_to_rad * left_diff_ticks,
            self.tick_to_rad * right_diff_ticks)

        if timestamp != self.last_timestamp:
            self.last_diff_time = (timestamp - self.last_timestamp) / 1000.0
            self.last_timestamp = timestamp
            self.last_velocity_left = (self.tick_to_rad * left_diff_ticks) / self.last_diff_time
            self.last_velocity_right = (self.tick_to_rad * right_diff_ticks) / self.last_diff_time

        pose_update_rates = [
            pose_update[0] / self.last_diff_time,  # x (m)
            pose_update[1] / self.last_diff_time,  # y (m)
            pose_update[2] / self.last_diff_time,  # heading (rad)
        ]

        return pose_update, pose_update_rates

    def pose_update_from_wheel_differential(self, dleft, dright):
        '''
        Local robot frame of reference has the x axis pointing forward.
        Since the pose update is using the robot frame of reference, the y update is zero.

        Source: https://github.com/stonier/ecl_core/blob/9d1c49c882c57668f3c46adb2a1a38c2d0957e59/ecl_mobile_robot/src/lib/differential_drive.cpp#L37
        '''
        ds = self.wheel_radius * (dleft + dright) / 2.0
        domega = self.wheel_radius * (dright - dleft) / self.bias
        return [ds, 0, wrap_angle(domega)]

    