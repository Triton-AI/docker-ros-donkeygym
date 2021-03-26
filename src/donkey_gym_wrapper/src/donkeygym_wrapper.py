#!/usr/bin/env python3
import gyminterface
from gyminterface import GymInterface
# from PIL import Image
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
# from catkin_ws.src.ocvfiltercar.scripts.filter_follow import LineFollower


class Wrapper:
    def __init__(self):
        GYM_DICT = self.load_param('/catkin_ws/src/donkey_gym_wrapper/config/config.yaml')
        print(GYM_DICT)

        self.gym = GymInterface(gym_config=GYM_DICT)

        self.img, _, _, _, _ ,_, _ = self.gym.step(0, 0, 0, 0)
        print(f"img: {self.img}")
        self.img = ImageTools.convert_cv2_to_ros_msg(self.img)
        print(f"img: {self.img}")

        self.drive_sub = rospy.Subscriber('/drive', AckermannDriveStamped, self.drive_callback)
        self.lidar_pub = rospy.Publisher('/lidar', LaserScan, queue_size=10)
        self.image_pub = rospy.Publisher('/image', self.img, queue_size=10)
        self.twist_pub = rospy.Publisher('/twist', Twist, queue_size=10)
        self.ImgT = ImageTools()
        # self.max_steering = self.load_param("~max_steering") # create a launch file
        self.last_speed = 0

    def drive_callback(self, drive):
        steering = (drive.drive.steering_angle / self.max_steering) * 180 / math.pi
        throttle = int(drive.drive.speed > self.last_speed)
        twist_msg = Twist()
        self.img, self.twist_msg.linear.x, self.twist_msg.linear.y, self.twist_msg.linear.z, self.last_speed, _, self.laser_msg = \
                                                        self.gym.step(steering, throttle, breaking, reset)
        Image = self.ImgT.convert_cv2_to_ros_msg(self.img)
        # print(lidar)

    def load_param(self, path):
        with open(path, "r") as file:
            return yaml.load(file, Loader=yaml.FullLoader)



    def pub(self):
        while True:
            if self.lidar_pub is not None:
                self.lidar_pub.publish()
            if self.image_pub is not None:
                # print(Image)
                self.image_pub.publish()
            if self.twist_pub is not None:
                self.twist_pub.publish()


def main():
    rospy.init_node("Wrapper_node", anonymous=True)
    # drive = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)
    # a = AckermannDriveStamped()
    # a.drive.speed = 4
    # a.drive.steering_angle = 1
    w = Wrapper()
    T = Thread(target=w.pub, daemon=False)
    T.start()
    # drive.publish(a)

    rospy.spin()


if __name__ == '__main__':
    main()
