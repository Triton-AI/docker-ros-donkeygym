#!/usr/bin/env python3
import gyminterface
from gyminterface import GymInterface
from PIL import Image
import json
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Twist
import math
import numpy as np
from threading import Thread
from image_tools import ImageTools


class Wrapper:
    def __init__(self):
        self.load_param('~config')

        self.gym = GymInterface(gym_config=GYM_DICT)
        self.drive_sub = rospy.Subscriber('/drive', AckermannDriveStamped, self.drive_callback)
        self.lidar_pub = rospy.Publisher('/lidar', LaserScan, queue_size=10)
        self.image_pub = rospy.Publisher('/image', Image, queue_size=10)
        self.twist_pub = rospy.Publisher('/twist', Twist, queue_size=10)
        self.ImgT = ImageTools()
        # self.max_steering = self.load_param("~max_steering") # create a launch file
        self.last_speed = 0

    def drive_callback(self, drive):
        steering = (drive.drive.steering_angle / self.max_steering) * 180 / math.pi
        throttle = int(drive.drive.speed > self.last_speed)
        twist_msg = Twist()
        self.img, self.twist_msg.linear.x, self.twist_msg.linear.y, self.twist_msg.linear.z, self.last_speed, _, self.laser_msg = self.gym.step(
            (steering, throttle, breaking, reset))
        Image = self.ImgT.convert_cv2_to_ros_msg(self.img)
        # print(lidar)

    def pub(self):
        while True:
            if self.lidar_pub is not None:
                self.lidar_pub.publish()
            if self.image_pub is not None:
                self.image_pub.publish(Image)
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