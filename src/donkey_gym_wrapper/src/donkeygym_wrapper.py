#!/usr/bin/env python3
import sys
import json
import yaml
import time
import rospy
import numpy as np
from threading import Thread
from collections import deque
from image_tools import ImageTools
from geometry_msgs.msg import Twist
from gyminterface import GymInterface
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class Wrapper:
    def __init__(self):
        GYM_DICT = self.load_param('/catkin_ws/src/donkey_gym_wrapper/config/config.yaml')

        self.gym = GymInterface(gym_config=GYM_DICT)

        self.drive_sub = rospy.Subscriber('/drive', AckermannDriveStamped, self.drive_callback)
        self.lidar_pub = rospy.Publisher('/lidar', LaserScan, queue_size=10)
        self.image_pub = rospy.Publisher('/image', Image, queue_size=10)
        self.twist_pub = rospy.Publisher('/twist', Twist, queue_size=10)
        self.ImgT = ImageTools()
        self.twist_msg = Twist()
        # self.breaking = deque()
        self.breaking = 10
        self.smooth_throttle = deque()

        angle_min = 0
        angle_max = 360
        ang_inc = self.gym.deg_inc / 180 * np.pi
        self.lidar = LaserScan(angle_min=angle_min, angle_max=angle_max, angle_increment=ang_inc, range_min=0, range_max=float("inf"))

        self.last_speed = 0
        self.called = False

    def drive_callback(self, drive):
        self.called = True
        breaking, reset = 0, 0
        self.max_steering = 1
        steering = (drive.drive.steering_angle / self.max_steering) * 180 / np.pi
        throttle = int(drive.drive.speed > self.last_speed)

        # sharp turn breaking
        # self.breaking.appendleft(throttle)
        # if len(self.breaking) > 250:
        #     self.breaking.pop()
        #     if self.breaking.count(1) < 5:
        #         print(self.breaking)
        #         print("Breaking!!!!!!!!!!!!")
        #         breaking = 1 / (self.breaking.count(1) + 3)
        if drive.drive.speed < -1 and self.last_speed > 2:
            self.breaking += 1
            breaking = 1 / (self.breaking * 0.1)
            print(f"Breaking time!! {breaking:.5f} {self.last_speed:.5f}")
        else:
            self.breaking = 0
        
        self.smooth_throttle.appendleft(throttle)
        if len(self.smooth_throttle) > 80:
            self.smooth_throttle.pop()
            throttle = sum(self.smooth_throttle) / 80
        
        # communicate with gyminterface
        self.img, _, _, _, self.last_speed, _, self.laser_msg = self.gym.step(steering, throttle, breaking, reset)

        # process data and publish 
        if self.laser_msg is not None:
            self.lidar.ranges = self.convert_lidar_to_laserscan(self.laser_msg)
        ros_img = self.ImgT.convert_cv2_to_ros_msg(self.img)

        self.lidar_pub.publish(self.lidar)
        self.image_pub.publish(ros_img)

    def load_param(self, path):
        with open(path, "r") as file:
            return yaml.load(file, Loader=yaml.FullLoader)
    
    def convert_lidar_to_laserscan(self, laser_msg):
        mapped_ranges = np.zeros((1, 360))[0]
        laser_msg = np.array(list(filter(lambda x: x["ry"] == 0, laser_msg)))
        mapped_ranges[np.array(list(map(lambda d: d["rx"], laser_msg)))] = np.array(list((map(lambda d: d["d"], laser_msg))))
        return mapped_ranges


def main():
    rospy.init_node("wrapper_node", anonymous=True)
    w = Wrapper()

    rospy.Rate(40)
    rospy.spin()
    def shutdownhook():
        print("Shutting down lolololol.....")
        sys.exit(1)

    rospy.on_shutdown(shutdownhook)


if __name__ == '__main__':
    main()
