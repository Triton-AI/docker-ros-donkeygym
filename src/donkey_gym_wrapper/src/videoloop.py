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
        self.image_pub = rospy.Publisher('/image', Image, queue_size=4)
        self.image_pub_b = rospy.Publisher('/imageb', Image, queue_size=10)
        self.twist_pub = rospy.Publisher('/twist', Twist, queue_size=10)
        self.ImgT = ImageTools()
        self.img, self.img_b = None, None
        self.smooth_steering = deque()
        self.smooth_throttle = deque()

        angle_min = 0
        angle_max = 360
        ang_inc = self.gym.deg_inc / 180 * np.pi
        self.lidar = LaserScan(angle_min=angle_min, angle_max=angle_max, angle_increment=ang_inc, range_min=0, range_max=float("inf"))

        self.last_speed = 0
        self.called = False
        print("\n\n")

    def drive_callback(self, drive):
        self.called = True
        breaking, reset = 0, 0
        self.max_steering = 1
        steering = (drive.drive.steering_angle / self.max_steering) * 180 / np.pi
        throttle = int(drive.drive.speed > self.last_speed)

        # Steering and Throttle control
        steering = steering if steering < 1 else 1
        steering_interval = 5
        self.smooth_steering.appendleft(steering)
        if len(self.smooth_steering) > steering_interval:
            self.smooth_steering.pop()
            steering = sum(self.smooth_steering) / steering_interval

        throttle_interval = 70
        self.smooth_throttle.appendleft(throttle)
        if len(self.smooth_throttle) > throttle_interval:
            self.smooth_throttle.pop()
            throttle = sum(self.smooth_throttle) / throttle_interval
        
        if self.last_speed < 0:
            print(f"Detected low speed, accelerating!! Speed: {self.last_speed:.2f}", end="\r", flush=True)
            throttle += 6 / (self.last_speed + 0.01)
        
        # communicate with gyminterface
        self.img, self.img_b, _, _, _, self.last_speed, _, self.laser_msg = self.gym.step(steering, throttle, breaking, reset)

        # process data and publish 
        if self.laser_msg is not None:
            self.lidar.ranges = self.convert_lidar_to_laserscan(self.laser_msg)
        
        if self.img is not None and self.img_b is not None:
            ros_img = self.ImgT.convert_cv2_to_ros_msg(self.img)
            ros_img_b = self.ImgT.convert_cv2_to_ros_msg(self.img_b)

            self.lidar_pub.publish(self.lidar)
            self.image_pub.publish(ros_img)
            self.image_pub_b.publish(ros_img_b)

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
    image_pub = rospy.Publisher('/image', Image, queue_size=4)
    image_tool = ImageTools()
    cv2.VideoCapture(0)
    while(True):
        ret, frame = vid.read()
        ros_img = self.ImgT.convert_cv2_to_ros_msg(frame)
        cv2.imshow('frame', frame)
        image_pub.publish(ros_img)
        
    def shutdownhook():
        print("Shutting down lolololol.....")
        sys.exit(1)

    rospy.on_shutdown(shutdownhook)
    vid.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
