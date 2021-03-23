#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import math
from os import path
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from move_robot import MoveRosBots
from geometry_msgs.msg import Twist

def callback(x):
    pass

class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.datapath = "/home/michaelji/tritonai/catkin_ws/src/ocvfiltercar/data/records_1/"

        #self.image_sub = rospy.Subscriber("/robot1/camera1/image_raw",Image,self.camera_callback)
        self.image_sub = rospy.Subscriber("/image", Image, self.camera_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

        self.moverosbots_object = MoveRosBots()

    def camera_callback(self,data):

        # HSV filter for isolating all lines
        bestfilter = {
            "lowH": 16,
            "highH": 43,
            "lowS": 51,
            "highS": 140,
            "lowV": 42,
            "highV": 213
        }
        # RGB for mask to isolate yellow center line
        rgbcenterfilter = {
            "lowR": 200,
            "highR": 255,
            "lowG": 100,
            "highG": 255,
            "lowB": 100,
            "highB": 170
        }
        # RGB for mask to isolate white borders
        rgbsidefilter = {
            "lowR": 200,
            "highR": 255,
            "lowG": 200,
            "highG": 255,
            "lowB": 200,
            "highB": 255
        }

        # Load filter values
        lowR = rgbcenterfilter.get("lowR")
        highR = rgbcenterfilter.get("highR")
        lowG = rgbcenterfilter.get("lowG")
        highG = rgbcenterfilter.get("highG")
        lowB = rgbcenterfilter.get("lowB")
        highB = rgbcenterfilter.get("highB")

        # Get image from IMG message
        try:
        # We select bgr8 because its the OpneCV encoding by default
            # DEBUG: cv_image = cv2.imread(self.datapath + "img_" + str(0) + ".jpg")

            cv_image = self.bridge_object.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            exit(1)
            
        # Crop image
        height, width, channels = cv_image.shape
        crop_img = cv_image[int(height/2):height, 0:width]

        rgb = cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB)

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

        error_x = cx - width / 2;
        angular_z = -error_x / 100;

        # ROS Message publish
        a = AckermannDriveStamped()

        a.drive.steering_angle = angular_z
        a.drive.speed = 2 / (1 + math.exp(abs(4 * angular_z)))

        rospy.loginfo("ANGULAR VALUE: " + str(a.drive.steering_angle))
        rospy.loginfo("SPEED VALUE: " + str(a.drive.speed))

        self.drive_pub.publish(a)

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
        self.moverosbots_object.clean_class()
        cv2.destroyAllWindows()
        
        

def main():
    rospy.init_node('line_following_node', anonymous=True)
    
    line_follower_object = LineFollower()
    
    line_follower_object.camera_callback(3)

    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()
    
    
if __name__ == '__main__':
    main()
