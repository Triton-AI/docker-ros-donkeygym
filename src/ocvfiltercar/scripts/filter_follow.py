#!/usr/bin/env python3
import rospy
import cv2
import math
import time
import numpy as np
from os import path
from simple_pid import PID
from threading import Thread
from sensor_msgs.msg import Image
from move_robot import MoveRosBots
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.datapath = "/home/michaelji/tritonai/catkin_ws/src/ocvfiltercar/data/records_1/"

        # ROS Message publish
        self.image_sub = rospy.Subscriber("/image", Image, self.camera_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

        # HSV filter for isolating all lines
        self.bestfilter = {
            "lowH": 16,
            "highH": 43,
            "lowS": 51,
            "highS": 140,
            "lowV": 42,
            "highV": 213
        }
        # RGB for mask to isolate yellow center line
        self.rgbcenterfilter = {
            "lowR": 200,
            "highR": 255,
            "lowG": 200,
            "highG": 255,
            "lowB": 0,
            "highB": 170
        }
        # RGB for mask to isolate white borders
        self.rgbsidefilter = {
            "lowR": 216,
            "highR": 255,
            "lowG": 222,
            "highG": 255,
            "lowB": 198,
            "highB": 255
        }

        # Misc
        self.running, self.isStart, self.a_drive = False, False, None
        self.ang_mul = 0.17

        self.a_drive = AckermannDriveStamped()
        self.a_drive.drive.steering_angle = 0.0
        self.a_drive.drive.speed = 0.0

        self.throttle_pid = PID(0.25, 0.0, 0.005)
        self.steering_pid = PID(0.1, 0.0005, 0.35)
        # self.throttle_pid.output_limits = (2, 1000)

        self.angular_z, self.speed, self.steering = 0, 0, 0

        self.speed1 = 0

    def dummy_publish(self):
        print("\n")
        while not self.running:
            print("Start to publish drive msg, wait for wrapper node to recieve!", end="\r")
            self.drive_pub.publish(self.a_drive)

        print("\n")
        cv2.namedWindow("Image")
        cv2.moveWindow("Image", 120,850)

        throttle_pid = PID(0.25, 0.0, 0.005)
        steering_pid = PID(0.1, 0.0005, 0.35)

        while 1:
            # rospy.loginfo(f"{self.speed1}")
            rospy.loginfo(f"Speed: {self.a_drive.drive.speed:.4f} Steering: {self.a_drive.drive.steering_angle:.4f}")
            

            cv2.circle(self.rgb,(int(self.cx), int(self.cy)), 5,(0,0,255),-1)
            cv2.imshow('Image', self.rgb)
            cv2.waitKey(30)

    def camera_callback(self,data):
        # Initialize
        self.running, self.isShow = True, True
        

        # Load filter values
        lowR = self.rgbcenterfilter.get("lowR")
        highR = self.rgbcenterfilter.get("highR")
        lowG = self.rgbcenterfilter.get("lowG")
        highG = self.rgbcenterfilter.get("highG")
        lowB = self.rgbcenterfilter.get("lowB")
        highB = self.rgbcenterfilter.get("highB")

        # Get image from IMG message
        try:
            # We select bgr8 because its the OpneCV encoding by default
            # DEBUG: cv_image = cv2.imread(self.datapath + "img_" + str(0) + ".jpg")
            cv_image = self.bridge_object.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e, data)
            exit(1)
            
        # Crop image
        height, width, channels = cv_image.shape
        rgb = cv_image[int(height/3):int(height* (3/4)), 0:width]
        self.rgb = rgb

        # Create filter mask
        lower = np.array([lowR, lowG, lowB])
        higher = np.array([highR, highG, highB])
        mask = cv2.inRange(rgb, lower, higher)

        # Calculate c_x, c_y
        # Center Line:
        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2
        self.cy, self.cx = cy, cx

        error_x = cx - width / 2
        self.angular_z = (error_x / 100) * self.ang_mul
        
        # Jump start!!!
        if self.isStart == False:
            self.a_drive.drive.steering_angle = self.angular_z * self.ang_mul
            self.a_drive.drive.speed = 2
            self.drive_pub.publish(self.a_drive)
            time.sleep(0.8)
            self.isStart = True
        
        

        
        self.steering = self.steering_pid(self.angular_z)
        

        self.a_drive.drive.steering_angle = -self.steering
        self.speed1 = 1 / (math.exp(abs(self.steering / 0.0827 * 10))) * 14

        # self.speed = -self.throttle_pid(self.speed1)
        self.speed = self.speed1

        self.a_drive.drive.speed = self.speed # if speed > 1 else 1

        # rospy.loginfo("ANGULAR VALUE: " + str(a.drive.steering_angle))
        # rospy.loginfo("SPEED VALUE: " + str(a.drive.speed))

        self.drive_pub.publish(self.a_drive)

        # SIDES
        # Use gradients to determine steering
        # Load filter values
        """
        lowR = rgbsidefilter.get("lowR")
        highR = rgbsidefilter.get("highR")
        lowG = rgbsidefilter.get("lowG")
        highG = rgbsidefilter.get("highG")
        lowB = rgbsidefilter.get("lowB")
        highB = rgbsidefilter.get("highB")

        border_lower = np.array([lowR, lowG, lowB])
        border_higher = np.array([highR, highG, highB])
        border_mask = cv2.inRange(rgb, border_lower, border_higher)
        grad_x = np.diff(border_mask, n = 1, axis = 0)
        all_indices = [[]]
        print(grad_x)
        for row in grad_x:
            indices = []
            for index in range(len(row)):
                if row[index] != 0:
                    print(str(index) + " ", end='')
                    np.append(indices, index)

            if len(indices) == 2:
                np.append(all_indices, indices)

        print(all_indices)
        print(np.mean(np.diff(all_indices, axis = 0), axis = 0))

        cv2.imshow('gradx', grad_x)
        cv2.moveWindow('gradx', 400, 600)
        """
        
    def clean_up(self):
        cv2.destroyAllWindows()
        

def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()

    # New thread
    t = Thread(target = line_follower_object.dummy_publish, daemon=False)
    t.start()

    rate = rospy.Rate(60)
    rospy.spin()

    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")

    rospy.on_shutdown(shutdownhook)
    
    
if __name__ == '__main__':
    main()
