import rclpy
from rclpy.time import Time
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math
import socket
import threading
from time import time
from typing import List

from .kobuki import *
from .defines import *
from .mobile_robot import MobileRobot
from .utils import *

class Kobuki(Node):
    stop_flag = False
    robot_data: TKobukiData = None
    mobile_robot = MobileRobot()

    odom = [0.0, 0.0, 0.0]

    def __init__(self):
        super().__init__('kobuki')

        self.setup_publishers()
        self.setup_subscribers()
        self.setup_timers()
        self.robot_setup_udp()
        self.get_logger().info(self.get_name() + " initialized")

    def __del__(self):
        self.robot_sock.close()

    def setup_publishers(self):
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

    def setup_subscribers(self):
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

    def setup_timers(self):
        self.kobuki_timer = self.create_timer(0.05, self.robot_timer_callback)
        self.cmd_vel_timer = self.create_timer(0.2, self.cmd_vel_timeout_callback)

    def robot_setup_udp(self):
        self.port = ROBOT_UDP_PORT
        self.robot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Message to be sent
        message = set_sound(440, 1000)
        self.send_udp_message(message)

        self.receiver_thread = threading.Thread(target=self.robot_udp_receiver_callback)
        self.receiver_thread.start()

    def cmd_vel_callback(self, msg: Twist):
        # Reset timeout of cmd_vel command
        self.cmd_vel_timer.reset()
        
        if math.isclose(msg.angular.z, 0.0):
            message = set_translation_speed(msg.linear.x * 1000)
        elif math.isclose(msg.linear.x, 0.0):
            message = set_rotation_speed(msg.angular.z)
        else:
            radius = (msg.linear.x * 1000) / msg.angular.z
            message = set_arc_speed(msg.linear.x * 1000, radius)
        self.send_udp_message(message)

    def robot_timer_callback(self):
        if not self.robot_data:
            return
        
        now = self.get_clock().now()

        self.publish_odometry(now)
    
    def cmd_vel_timeout_callback(self):
        '''Stop the robot when cmd_vel command does not arrive for 0.2s.'''
        message = set_translation_speed(0)
        self.send_udp_message(message)

    def robot_udp_receiver_callback(self):
        while not self.stop_flag:
            response, _ = self.robot_sock.recvfrom(1024)
            self.robot_data = parse_kobuki_message(response)
        print("UDP receiver stopped")
    
    def send_udp_message(self, message: List[int]):
        message_bytes = bytes(message)
        self.robot_sock.sendto(message_bytes, (UDP_IP, ROBOT_UDP_PORT))

    '''
    Based on https://github.com/kobuki-base/kobuki_core/blob/e2f0feac0f7a9964d021ac3241b7663f7728d5b9/src/driver/diff_drive.cpp#L51
    '''
    def publish_odometry(self, stamp: Time):
        # Calculate new robot state 
        pose_update, pose_update_rates = self.mobile_robot.update_diff_drive(
            self.robot_data.timestamp,
            self.robot_data.EncoderLeft,
            self.robot_data.EncoderRight)
    
        # Update odometry
        if not math.isclose(pose_update[0], 0.0):
            self.odom[0] += math.cos(self.odom[2]) * pose_update[0]
            self.odom[1] += math.sin(self.odom[2]) * pose_update[0]
        self.odom[2] = wrap_angle(self.odom[2] + pose_update[2])

        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        # Position
        msg.pose.pose.position.x = self.odom[0]
        msg.pose.pose.position.y = self.odom[1]
        msg.pose.pose.position.z = 0.0
        # Orientation
        quat = yaw_to_quaternion(self.odom[2])
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        # Linear velocity
        msg.twist.twist.linear.x = pose_update_rates[0]
        msg.twist.twist.linear.y = pose_update_rates[1]
        msg.twist.twist.linear.z = 0.0
        # Angular velocity
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = pose_update_rates[2]

        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    try:
        kobuki = Kobuki()
        rclpy.spin(kobuki)
    except KeyboardInterrupt:
        pass
    finally:
        kobuki.stop_flag = True

        # Destroy the node explicitly
        kobuki.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()