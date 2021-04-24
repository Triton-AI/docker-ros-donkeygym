#!/usr/bin/env python3
import cv2
import sys
import math
import time
import rospy
import numpy as np
from os import path
from simple_pid import PID
from threading import Thread
from move_robot import MoveRosBots
from geometry_msgs.msg import Twist
from timeit import default_timer as timer
from sklearn.mixture import GaussianMixture
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class LineFollower(object):
    def __init__(self):
        # ROS Message publish
        self.image_sub = rospy.Subscriber("/image", Image, self.camera_callback)
        self.image_sub_b = rospy.Subscriber("/imageb", Image, self.camera_callback_b)
        self.lidar_sub = rospy.Subscriber('/lidar', LaserScan, self.lidar_callback)
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
        self.center_lower = np.array([self.rgbcenterfilter["lowR"], self.rgbcenterfilter["lowG"], self.rgbcenterfilter["lowB"]])
        self.center_higher = np.array([self.rgbcenterfilter["highR"],self.rgbcenterfilter["highG"], self.rgbcenterfilter["highB"]])
        self.border_lower = np.array([self.rgbsidefilter["lowR"], self.rgbsidefilter["lowG"], self.rgbsidefilter["lowB"]])
        self.border_higher = np.array([self.rgbsidefilter["highR"], self.rgbsidefilter["highG"], self.rgbsidefilter["highB"]])

        # Misc
        self.bridge_object = CvBridge()
        self.dir, self.ct = None, 0
        self.running, self.isStart, self.a_drive = False, False, None
        self.angular_z, self.speed, self.steering, self.ang_mul = 0, 0, 0, 0.3  #  0.36
        self.a_drive = AckermannDriveStamped()
        self.a_drive.drive.steering_angle = 0.0
        self.a_drive.drive.speed = 0.0
        self.throttle_pid = PID(0.1, 0.0, 0.005)
        self.steering_pid = PID(0.1, 0.001, 0.2)
        self.steering_pid.auto_mode = True
        self.last_error, self.last_steering, self.last_speed = None, None, None
        self.last_spot, self.last_border_x = None, None
        self.end = False
        self.is_adjusted = 0

    def dummy_publish(self):
        # Before start
        print("\n")
        while not self.running:
            print("Start to publish drive msg, wait for wrapper node to recieve!", end="\r")
            self.drive_pub.publish(self.a_drive)

        # After start
        print("\n")
        cv2.namedWindow("ylo")
        cv2.moveWindow("ylo", 120,700)
        cv2.namedWindow("wte")
        cv2.moveWindow("wte", 120,870)
        while not self.end:
            ######################
            ##### Camera_b testing
            rgb = cv2.cvtColor(self.bgr_b, cv2.COLOR_BGR2RGB)
            mask = cv2.inRange(self.bgr_b, self.border_lower, self.border_higher)
            canny = cv2.GaussianBlur(cv2.Canny(mask, 100, 70), (3, 3), cv2.BORDER_DEFAULT)  # Blur
            lines = cv2.HoughLinesP(canny, 1, np.pi/180, threshold=40, minLineLength=15, maxLineGap=600)
            slope_arr = np.array([])
            x_value = np.array([])
            if lines is not None:
                for l in lines:
                    x1, y1, x2, y2 = l[0]
                    if x2 - x1 != 0:
                        slope_arr = np.append(slope_arr, -(y2 - y1) / (x2 - x1))
                    
                    x_value = np.append(x_value, x1)
                    x_value = np.append(x_value, x2)
                    pts = np.array([[x1, y1], [x2 , y2]], np.int32)
                    cv2.polylines(rgb, [pts], True, (0,255,0))
            if slope_arr is not None:
                std_distro = np.std(slope_arr)
                if std_distro < 1:
                    self.ct += 1 if self.ct < 30 else 0
                    self.dir = 0 if np.mean(x_value) < 110 else 1
                    d = "Left" if self.dir == 0 else "Right"
                    print(f"{d} turn detected!!!!! Speed: {self.speed:.3f}")
                else:
                    self.ct -= 1.3 if self.ct > 1 else 0

            cv2.imshow('ylo', rgb)
            
            ########################
            ##### White line testing
            # cv2.fillPoly(self.canny, pts = [self.contours], color = (255, 255, 255))
            rgb = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2RGB)
            cv2.circle(rgb, (int(self.centroid[0]), int(self.centroid[1])), 5, (0, 0, 0), -1)
            # rospy.loginfo(f"Speed: {self.a_drive.drive.speed:.4f} Steering: {self.a_drive.drive.steering_angle:.4f}")
            cv2.imshow("wte", rgb)
            cv2.waitKey(60)
    
    def lidar_callback(self, lidar_msg):
        self.ranges = lidar_msg.ranges
    
    def camera_callback_b(self, data):
        self.bgr_b = self.bridge_object.imgmsg_to_cv2(data, "bgr8")
        height, width, channels = self.bgr_b.shape
        self.bgr_b = self.bgr_b[int(height * (2 / 5)):int(height), :]
        
    def camera_callback(self, data):
        # Initialize
        self.running, self.isShow = True, True

        # Get image from IMG message
        cv_image = self.bridge_object.imgmsg_to_cv2(data, "bgr8")  # We select bgr8 because its the OpneCV encoding by default

        # Crop image
        height, width, channels = cv_image.shape
        self.bgr = cv_image[int(height * (2 / 5)):int(height * (3 / 4)), :]
        
        # Use white lanes
        error_x, ang = self.extract_white_line()
        if ang:
            if self.is_adjusted == 3:
                print("Both!!!!!!")
            self.steering = ang
        else:
        # Calculate speed and steering values
            self.angular_z = (error_x / 100) * self.ang_mul
            self.steering = self.steering_pid(self.angular_z)
            if not self.last_steering or abs(self.last_steering - self.steering) < 0.01:  # steering stability control
                self.last_steering = self.steering
            else:
                self.steering = self.last_steering
        
        # Speed control on turns
        if 28 > self.ct > 0:
            if self.dir == 0:
                self.steering += 0.0014
            elif self.dir == 1:
                self.steering -= 0.0014
            self.speed = 1 / (math.exp(abs(self.steering / 0.048 * 10))) * 16 - 2
        elif self.ct > 28:
            if self.dir == 0:
                self.steering += 0.00335
            elif self.dir == 1:
                self.steering -= 0.00335
            self.speed = 1 / (math.exp(abs(self.steering / 0.048 * 10))) * 45 - 2
            
        else:
            self.speed = 1 / (math.exp(abs(self.steering / 0.048 * 10))) * 50 - 2
        
        # Publish drive message
        self.a_drive.drive.steering_angle = -self.steering
        self.a_drive.drive.speed = self.speed
        self.drive_pub.publish(self.a_drive)

    def extract_white_line(self):
        mask = cv2.inRange(self.bgr, self.border_lower, self.border_higher)
        self.canny = cv2.GaussianBlur(cv2.Canny(mask, 100, 70), (3, 3), cv2.BORDER_DEFAULT)  # canny + Blur
        lines = cv2.HoughLinesP(self.canny, 1, np.pi/180, threshold=40, minLineLength=15, maxLineGap=600)

        # Set initial maximum and minimum slopes and lines
        max_slope = 0
        min_slope = 0
        max_slope_line = None
        min_slope_line = None

        # For each line, best left line is max slope, best right line is min slope
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x2 - x1 != 0:
                    slope = - (y2 - y1) / (x2 - x1)
                    if slope > max_slope:
                        max_slope = slope
                        max_slope_line = line
                    elif slope < min_slope:
                        min_slope = slope
                        min_slope_line = line

        steering_ang = None
        # Draw lines
        self.is_adjusted = 0
        if max_slope_line is None:
            max_slope_line = np.array([[0, mask.shape[0], 0, 0]])
            steering_ang = 0.5
            self.is_adjusted += 1
        if min_slope_line is None:
            min_slope_line = np.array([[mask.shape[1], 0, mask.shape[1], mask.shape[0]]])
            steering_ang = -0.5
            self.is_adjusted += 2

        # find centroid
        self.centroid = ((max_slope_line[0][0] + max_slope_line[0][2] 
            + min_slope_line[0][0] + min_slope_line[0][2]) / 4,
            (max_slope_line[0][1] + max_slope_line[0][3] 
            + min_slope_line[0][1] + min_slope_line[0][3]) / 4)
        
        self.contours = np.array([[max_slope_line[0][0], max_slope_line[0][1]],
                [max_slope_line[0][2], max_slope_line[0][3]], 
                [min_slope_line[0][0], min_slope_line[0][1]],
                [min_slope_line[0][2], min_slope_line[0][3]]])

        return self.centroid[0] - mask.shape[1] / 2, steering_ang

    def clean_up(self):
        self.end = True
        cv2.destroyAllWindows()
        sys.exit(1)
        

def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
    # New thread
    t = Thread(target = line_follower_object.dummy_publish, daemon=False)
    t.start()

    """
    rfg = reactive_follow_gap()
    # New thread
    t = Thread(target = rfg.dummy_publish, daemon=False)
    t.start()
    """
    rate = rospy.Rate(10)
    rospy.spin()

    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")

    rospy.on_shutdown(shutdownhook)
    
    
if __name__ == '__main__':
    main()

################################################################################################
################################################################################################
################################################################################################

class reactive_follow_gap:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber('/lidar', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        self.stored_ranges = []

        self.a_drive = AckermannDriveStamped()
        self.a_drive.drive.steering_angle = 0.0
        self.a_drive.drive.speed = 0.0
        self.running = False

    def dummy_publish(self):
        # Before start
        print("Dummy")
        while not self.running:
            print("Start to publish drive msg, wait for wrapper node to recieve!", end="\r")
            self.drive_pub.publish(self.a_drive)
        
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        #temp_ranges = list(ranges)
        #quarter_len = len(temp_ranges) / 4
        #proc_ranges = temp_ranges[quarter_len:(len(temp_ranges) - quarter_len)]
        proc_ranges = list(ranges)
        max_accepted_distance = 15
        
        for i in range(len(proc_ranges)):
            if (math.isnan(ranges[i])):
                proc_ranges[i] = 0
            elif ((ranges[i] > max_accepted_distance) or math.isinf(ranges[i])):
                proc_ranges[i] = max_accepted_distance
            else:
                proc_ranges[i] = ranges[i]
        
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        
        max_begin = 0
        current_gap = 0
        current_begin = 0
        current_index = 0
        max_gap = 0

        while(current_index < len(free_space_ranges)):
            
            current_gap = 0
            current_begin = current_index

            while ((current_index < len(free_space_ranges)) 
                and (free_space_ranges[current_index] > 0.5)):
                current_gap+=1
                current_index+=1
            
            if (current_gap > max_gap):
                max_gap = current_gap
                max_begin = current_begin
                current_gap = 0

            current_index+=1

        if (current_gap > max_gap):
            max_gap = current_gap
            max_begin = current_begin

        return max_begin, max_begin + max_gap - 1 
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """

        avg = np.average(ranges[start_i:end_i], weights=ranges[start_i:end_i])
        
        best_point = start_i + np.argmin(np.abs(np.array(ranges[start_i:end_i]) - avg))

        return best_point

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm 
        & publish an AckermannDriveStamped Message
        """
        self.runing = True
        array = np.zeros((1, 1080))
        for i in data.ranges:
            array[int(i["rx"]) * 3] = i["d"]

        ranges = array
        proc_ranges = self.preprocess_lidar(ranges)
        
        #Find closest point to LiDAR
        closest_point = np.argmin(proc_ranges)


        #Eliminate all points inside 'bubble' (set them to zero) 
        ratio_of_bubble_to_ranges = 15
        bubble_radius = (len(proc_ranges) / ratio_of_bubble_to_ranges) / 2
        for i in range(int(max(0, closest_point - bubble_radius)), 
	        int(min(closest_point + bubble_radius, len(proc_ranges) - 1))):
            proc_ranges[i] = 0

        #Find max length gap 
        start_point, end_point = self.find_max_gap(proc_ranges)

        #Find the best point in the gap 
        best_point = self.find_best_point(start_point, end_point, proc_ranges)

        #Steering angle
        best_steering_angle = 0
        angle_increment = 0.002
        if (best_point < len(proc_ranges) / 2):
            distance = (len(proc_ranges) / 2) - best_point
            best_steering_angle = - distance * angle_increment
        else:
            distance = (best_point - (len(proc_ranges) / 2))
            best_steering_angle = distance * angle_increment

        #Speed Calculation


        #Publish Drive message
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = "laser"
        ack_msg.drive.steering_angle = best_steering_angle
        
        multiplier = 3.5
        multiplier2 = 0.3
        pred_speed = abs((1.8 * multiplier) - ((6 * abs(best_steering_angle)) ** 2))
        delta = abs(pred_speed - ack_msg.drive.speed)
        speed1 = multiplier * math.atan(delta * multiplier2)
        ack_msg.drive.speed = speed1
        

        self.convert_data(ack_msg.drive.speed, ack_msg.drive.steering_angle)
        # ack_msg.drive.acceleration = 2
        self.drive_pub.publish(ack_msg)