#!/usr/bin/env python3
import gyminterface
from gyminterface import GymInterface
import json
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Twist
import math
import numpy as np
from threading import Thread
from image_tools import ImageTools
import yaml
import time
import matplotlib.pyplot as plt


class Wrapper:
    def __init__(self):
        GYM_DICT = self.load_param('/catkin_ws/src/donkey_gym_wrapper/config/config.yaml')

        self.gym = GymInterface(gym_config=GYM_DICT)

        self.drive_sub = rospy.Subscriber('/drive', AckermannDriveStamped, self.drive_callback)
        self.lidar_pub = rospy.Publisher('/lidar', LaserScan, queue_size=10)
        self.image_pub = rospy.Publisher('/image', Image, queue_size=10)
        self.twist_pub = rospy.Publisher('/twist', Twist, queue_size=10)
        self.ImgT = ImageTools()
        self.last_speed = 0
        self.called = False

    def drive_callback(self, drive):
        self.called = True
        breaking, reset = 0, 0
        self.max_steering = 1
        steering = (drive.drive.steering_angle / self.max_steering) * 180 / math.pi
        throttle = int(drive.drive.speed > self.last_speed)
        self.twist_msg = Twist()
        self.img, self.twist_msg.linear.x, self.twist_msg.linear.y, self.twist_msg.linear.z, self.last_speed, _, self.laser_msg = \
                                                        self.gym.step(steering, throttle, breaking, reset)
        Image2 = self.ImgT.convert_cv2_to_ros_msg(self.img)

        msg = LaserScan(ranges=np.array(self.laser_msg))

        # index = [i["rx"] / 2 for i in self.laser_msg]
        # distance = [i["rx"] / 2 for i in self.laser_msg]

        if self.lidar_pub is not None:
            self.lidar_pub.publish()
        if self.image_pub is not None:
            self.image_pub.publish(Image2)
        if self.twist_pub is not None:
            self.twist_pub.publish()
        """
        Subscribe to one of the following topics:

        rgb/image_rect_color: Color rectified image (left sensor by default)
        rgb_raw/image_raw_color: Color unrectified image (left sensor by default)
        right/image_rect_color: Right camera rectified image
        right_raw/image_raw_color: Right camera unrectified image
        """

    def load_param(self, path):
        with open(path, "r") as file:
            return yaml.load(file, Loader=yaml.FullLoader)
    
    def convert_lidar_to_laserscan(self, laser_msg):
        scan_arr = np.array(laser_msg)
        degs_arr = scan_arr[np.arange(len(scan_arr))]['rx']
        dist_arr = scan_arr[np.arange(len(scan_arr))]['d']

        angle_min = 0
        angle_max = 360

        
        deg = list(map(lambda d: d["rx"], a))
        distance = list(map(lambda d: d["d"], a))

        return LaserScan()
    
    def print_lidar(self):
        while not self.called:
            pass
        plt.axes(projection = 'polar')
        plt.ion()
        plt.show()
        while 1:
            for i in self.laser_msg:
                print(self.laser_msg, type(self.laser_msg))
                # plt.polar((float(i["rx"]) * np.pi / 180), i["d"])
                # plt.pause(1)
                # plt.clf()


def main():
    rospy.init_node("wrapper_node", anonymous=True)
    w = Wrapper()
    t = Thread(target = w.print_lidar, daemon=False)
    t.start()

    rospy.Rate(60)
    rospy.spin()


if __name__ == '__main__':
    main()
