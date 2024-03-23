import rclpy
from rclpy.time import Time
from rclpy.node import Node

from nav_msgs.msg import Odometry

import math
import socket
import threading
from time import sleep, time
from operator import add

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
        self.setup_timers()
        self.robot_setup_udp()
        self.get_logger().info(self.get_name() + " initialized")

    def __del__(self):
        self.robot_sock.close()

    def setup_publishers(self):
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

    def setup_timers(self):
        self.kobuki_timer = self.create_timer(0.05, self.robot_timer_callback)

        # FIXME: remove me
        self.debug_timer_start = time()
        self.debug_timer = self.create_timer(0.1, self.debug_timer_callback)

    def robot_setup_udp(self):
        self.port = ROBOT_UDP_PORT
        self.robot_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Message to be sent
        message = set_sound(440, 1000)
        message_bytes = bytes(message)
        # Send the message to the server
        self.robot_sock.sendto(message_bytes, (UDP_IP, ROBOT_UDP_PORT))

        self.receiver_thread = threading.Thread(target=self.robot_udp_receiver_callback)
        self.receiver_thread.start()

    def robot_timer_callback(self):
        if not self.robot_data:
            return
        
        now = self.get_clock().now()

        self.publish_odometry(now)

    def debug_timer_callback(self):
        if not hasattr(self, 'debug_iter'):
            self.debug_iter = 0.0
        else:
            self.debug_iter += 0.2

        # Send command
        if (time() - self.debug_timer_start) > 5.0:
            self.debug_timer.cancel()
            message = set_translation_speed(0)
        else:
            message = set_translation_speed(int(math.sin(self.debug_iter) * 1000) & 0xffff)
        message_bytes = bytes(message)
        self.robot_sock.sendto(message_bytes, (UDP_IP, ROBOT_UDP_PORT))

    def robot_udp_receiver_callback(self):
        while not self.stop_flag:
            response, _ = self.robot_sock.recvfrom(1024)
            self.robot_data = parse_kobuki_message(response)
            # print(kobuki_data.EncoderLeft)
            # print(kobuki_data.EncoderRight)
            # print('-------')
            # sleep(0.01)
        print("While loop ended")
    
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
        self.odom = list(map(add, self.odom, pose_update))
        self.odom[2] = wrap_angle(self.odom[2])

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