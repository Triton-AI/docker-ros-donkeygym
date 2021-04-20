#!/usr/bin/env python3
import json
import rospy
import math
import yaml
import time
import numpy as np
from threading import Thread
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
        steering = (drive.drive.steering_angle / self.max_steering) * 180 / math.pi
        throttle = int(drive.drive.speed > self.last_speed)
        self.twist_msg = Twist()
        self.img, self.twist_msg.linear.x, self.twist_msg.linear.y, self.twist_msg.linear.z, self.last_speed, _, self.laser_msg = \
                                                        self.gym.step(steering, throttle, breaking, reset)

        ros_img = self.ImgT.convert_cv2_to_ros_msg(self.img)

        self.lidar.ranges = self.convert_lidar_to_laserscan(self.laser_msg)

        if self.lidar_pub is not None:
            self.lidar_pub.publish(self.lidar)
        if self.image_pub is not None:
            self.image_pub.publish(ros_img)
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
        mapped_ranges = np.zeros((1, 360))[0]
        laser_msg = np.array(list(filter(lambda x: x["ry"] == 0, laser_msg)))
        mapped_ranges[np.array(list(map(lambda d: d["rx"], laser_msg)))] = np.array(list((map(lambda d: d["d"], laser_msg))))
        return mapped_ranges


def main():
    rospy.init_node("wrapper_node", anonymous=True)
    w = Wrapper()
    # t = Thread(target = w.print_lidar, daemon=False)
    # t.start()

    rospy.Rate(100)
    rospy.spin()
    rospy.on_shutdown(lambda: print("Shutting down lolololol....."))


if __name__ == '__main__':
    main()
