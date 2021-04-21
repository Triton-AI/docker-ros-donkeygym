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
        self.bridge_object = CvBridge()
        # self.datapath = "/home/michaelji/tritonai/catkin_ws/src/ocvfiltercar/data/records_1/"

        # ROS Message publish
        self.image_sub = rospy.Subscriber("/image", Image, self.camera_callback)
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

    def lidar_callback(self, lidar_msg):
        self.ranges = lidar_msg.ranges

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
            ##### Yellow line
            # # cv2.circle(self.bgr,(int(self.cx), int(self.cy)), 5,(0,0,255),-1)
            rgb = cv2.cvtColor(self.bgr, cv2.COLOR_BGR2RGB)

            # mask = cv2.inRange(self.bgr, self.center_lower, self.center_higher)
            # result =cv2.bitwise_and(self.bgr, self.bgr, mask=mask)
            # cv2.circle(result,(int(self.cx), int(self.cy)), 5,(0,0,255),-1)
            cv2.imshow('ylo', rgb)

            
            ##### White line testing
            mask = cv2.inRange(self.bgr, self.border_lower, self.border_higher)
            canny = cv2.GaussianBlur(cv2.Canny(mask, 100, 70), (3, 3), cv2.BORDER_DEFAULT)  # Blur
            lines = cv2.HoughLinesP(canny, 1, np.pi/180, threshold=40, minLineLength=15, maxLineGap=600)
            # rospy.loginfo(f"{self.ranges}")
            
            """
            mid_y = mask.shape[0] // 2
            spot = np.where(canny[mid_y, :] == 255)[0]
            if len(spot) < 1:
                spot = self.last_spot
            self.last_spot = spot

            if np.ptp(spot) < 50:  # One edge is invisible (extreme cases)
                if self.last_border_x > mask.shape[1] // 2:
                    mid_x = (mask.shape[1] + (np.max(spot) + np.min(spot)) // 2) // 2  # Right extreme
                else:
                    mid_x = (np.max(spot) + np.min(spot)) // 4  # Left extreme
                # rospy.loginfo(f"{mid_x}, {mask.shape[1]}, {spot}")
            else:
                mid_x = (np.max(spot) + np.min(spot)) // 2
                if not self.last_border_x or abs(self.last_border_x - mid_x) < 25:
                    self.last_border_x = mid_x
                else:
                    mid_x = self.last_border_x
                    # rospy.loginfo(mid_x)
                # rospy.loginfo(f"{spot}, {(np.max(spot) + np.min(spot)) // 2}")


            cv2.circle(canny, (int(mid_x), int(mid_y)), 5, (255, 0, 0), -1)
            
            
            y_left_max = []
            y_right_max = []
            if lines is not None:
                for line in lines:
                    if line[0][0] < mask.shape[1] / 2 and line[0][1] > 25:  # left line
                        x1, y1, x2, y2 = line[0]
                        if len(y_left_max) and abs(y1 - y2) > abs(y_left_max[1] - y_left_max[3]):
                            y_left_max = line[0]
                        elif len(y_left_max) == 0:
                            y_left_max = line[0]
                    if line[0][0] > mask.shape[1] / 2 and line[0][1] < 25:  # right line
                        x1, y1, x2, y2 = line[0]
                        if len(y_right_max) and abs(y1 - y2) > abs(y_right_max[1] - y_right_max[3]):
                            y_right_max = line[0]
                        elif len(y_right_max) == 0:
                            y_right_max = line[0]

                    cv2.line(canny, (line[0][0], line[0][1]), (line[0][2], line[0][3]), (255, 0, 0), 3)

            if len(y_left_max):
                cv2.line(canny, (y_left_max[0], y_left_max[1]), (y_left_max[2], y_left_max[3]), (255, 0, 0), 3)
            if len(y_right_max):
                cv2.line(canny, (y_right_max[0], y_right_max[1]), (y_right_max[2], y_right_max[3]), (255, 0, 0), 3)
            
            rospy.loginfo(f"{y_left_max}\n{y_right_max}")

            """
            s = timer()

            # Set initial maximum and minimum slopes and lines
            max_slope = 0
            min_slope = 0
            max_slope_line = None
            min_slope_line = None

            # For each line, best left line is max slope, best right line is min slope
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    slope = -(y2 - y1) / (x2 - x1)
                    if slope > max_slope:
                        max_slope = slope
                        max_slope_line = line
                    elif slope < min_slope:
                        min_slope = slope
                        min_slope_line = line

            # Draw lines
            if max_slope_line is None:
                max_slope_line = np.array([[0, mask.shape[0], 0, 0]])
            if min_slope_line is None:
                min_slope_line = np.array([[mask.shape[1], 0, mask.shape[1], mask.shape[0]]])
            cv2.line(canny, (max_slope_line[0][0], max_slope_line[0][1]),
                   (max_slope_line[0][2], max_slope_line[0][3]), (255, 0, 0), 3)
            cv2.line(canny, (min_slope_line[0][0], min_slope_line[0][1]),
               (min_slope_line[0][2], min_slope_line[0][3]), (255, 0, 0), 3)
            # rospy.loginfo(f"{max_slope_line}\n{min_slope_line}")


            # Contour and draw poly
            contours = np.array([[max_slope_line[0][0], max_slope_line[0][1]],
                [max_slope_line[0][2], max_slope_line[0][3]], 
                [min_slope_line[0][0], min_slope_line[0][1]],
                [min_slope_line[0][2], min_slope_line[0][3]]])

            centroid = ((max_slope_line[0][0] + max_slope_line[0][2] 
                + min_slope_line[0][0] + min_slope_line[0][2]) // 4,
                (max_slope_line[0][1] + max_slope_line[0][3] 
                + min_slope_line[0][1] + min_slope_line[0][3]) // 4)

            # cv2.fillPoly(canny, pts = [contours], color = (255, 255, 255))
            cv2.circle(canny, centroid, 5, (255, 255, 255), -1)

            # rospy.loginfo(f"{centroid[0] - mask.shape[1] /2}")
            rospy.loginfo(f"Speed: {self.a_drive.drive.speed:.4f} Steering: {self.a_drive.drive.steering_angle:.4f}\nTime: {(timer() - s):.4f}")
            
            cv2.imshow("wte", canny)
            cv2.waitKey(100)

    def camera_callback(self,data):
        # Initialize
        self.running, self.isShow = True, True

        # Get image from IMG message
        cv_image = self.bridge_object.imgmsg_to_cv2(data, "bgr8")  # We select bgr8 because its the OpneCV encoding by default

        # Crop image
        height, width, channels = cv_image.shape
        self.bgr = cv_image[int(height * (2 / 5)):int(height * (3 / 4)), :]

        # Using yellow line
        """
        # Create filter mask
        mask = cv2.inRange(self.bgr, self.center_lower, self.center_higher)

        # Calculate centroid of the blob of binary image using ImageMoments --> ERROR
        m = cv2.moments(mask, False)
        try:
            self.cx, self.cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            self.cy, self.cx = height / 2, width / 2
        """
        
        # Using white line
        """
        mask = cv2.inRange(self.bgr, self.border_lower, self.border_higher)
        canny = cv2.Canny(mask, 100, 100)

        mid_y = mask.shape[0] // 2
        spot = np.where(canny[mid_y, :] == 255)[0]
        if len(spot) < 1:
            spot = self.last_spot
        self.last_spot = spot
        

        #if self.last_border_x is None:
        #    self.last_border_x = mask.shape[1] // 2
        if np.ptp(spot) < 50:  # One edge is invisible (extreme cases)
            if self.last_border_x > mask.shape[1] // 2:
                mid_x = (mask.shape[1] + (np.max(spot) + np.min(spot)) // 2) // 2  # Right extreme
            else:
                mid_x = (np.max(spot) + np.min(spot)) // 4  # Left extreme
            # rospy.loginfo(f"{mid_x}, {mask.shape[1]}, {spot}")
        else:
            mid_x = (np.max(spot) + np.min(spot)) // 2
            if not self.last_border_x or abs(self.last_border_x - mid_x) < 25:
                self.last_border_x = mid_x
            else:
                mid_x = self.last_border_x

        error_x = mid_x - width / 2
        if not self.last_error or abs(error_x - self.last_error) < 80:  # error stability control
            self.last_error = error_x
        else:
            error_x = self.last_error
        """
        
        error_x, ang = self.extract_white_line()
        if ang:
            self.steering = ang
            self.a_drive.drive.steering_angle = self.steering
        else:
        # Calculate speed and steering values
            self.angular_z = (error_x / 100) * self.ang_mul
            self.steering = self.steering_pid(self.angular_z)
            if not self.last_steering or abs(self.last_steering - self.steering) < 0.01:  # steering stability control
                self.last_steering = self.steering
            else:
                self.steering = self.last_steering
            self.a_drive.drive.steering_angle = -self.steering


        self.speed = 1 / (math.exp(abs(self.steering / 0.048 * 10))) * 19 - 2
        # if not self.last_speed or abs(self.speed - self.last_speed) < 13:  # speed stability control
        #     self.last_speed = self.speed
        # else:
        #     self.speed = self.last_speed
        self.a_drive.drive.speed = self.speed
        
        # Publish drive message
        self.drive_pub.publish(self.a_drive)

    def extract_white_line(self):
        mask = cv2.inRange(self.bgr, self.border_lower, self.border_higher)
        canny = cv2.GaussianBlur(cv2.Canny(mask, 100, 70), (3, 3), cv2.BORDER_DEFAULT)  # canny + Blur
        lines = cv2.HoughLinesP(canny, 1, np.pi/180, threshold=40, minLineLength=15, maxLineGap=600)

        # Set initial maximum and minimum slopes and lines
        max_slope = 0
        min_slope = 0
        max_slope_line = None
        min_slope_line = None

        # For each line, best left line is max slope, best right line is min slope
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = - (y2 - y1) / (x2 - x1)
                if slope > max_slope:
                    max_slope = slope
                    max_slope_line = line
                elif slope < min_slope:
                    min_slope = slope
                    min_slope_line = line

        steering_ang = None
        # Draw lines
        if max_slope_line is None:
            max_slope_line = np.array([[0, mask.shape[0], 0, 0]])
            steering_ang = -0.5
        if min_slope_line is None:
            min_slope_line = np.array([[mask.shape[1], 0, mask.shape[1], mask.shape[0]]])
            steering_ang = 0.5

        # find centroid
        centroid = ((max_slope_line[0][0] + max_slope_line[0][2] 
            + min_slope_line[0][0] + min_slope_line[0][2]) / 4,
            (max_slope_line[0][1] + max_slope_line[0][3] 
            + min_slope_line[0][1] + min_slope_line[0][3]) / 4)

        return centroid[0] - mask.shape[1] / 2, steering_ang

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
    rate = rospy.Rate(40)
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
