#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import json

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Twist


class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        # lidarscan_topic = '/scan'
        # drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber('image', LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=100)
        self.stored_ranges = []

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

        # Bad:
        # best_point = start_i + np.argsort(ranges[start_i:end_i])[len(ranges[start_i:end_i])//2]

        global ct1, m
        ct1 += 1
        if ct1 > m:
            print("\n\n")
            print("Average: ", np.mean(ranges[start_i:end_i]))
            print("Weighted average:", avg)
            print("Start index: " + str(start_i) + "\nEnd index: " + str(end_i) + "\nBest_point: " + str(best_point))
            ct1 = 0

        return best_point


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm 
        & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
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


        # print statement
        global ct, m
        ct += 1
        if ct > m:
            print("Speed: ", ack_msg.drive.speed)
            print("Predicted speed: ", pred_speed)
            print("Steering angle: ", best_steering_angle)
            print("Bestpoint: ", best_point)
            print("Angle increment(under data): ", data.angle_increment)
            print("Distance: (" + str(len(proc_ranges)) + " / 2) - " + str(best_point) +  " = " + str(distance))
            print("\n")
            self.print_ranges(proc_ranges)
            ct = 0
            


    def convert_data(self, throttle, steering):
        datas = {
            "throttle": throttle,
            "steering": steering / 1.08,
            "breaking": None,
            "reset": None
        }
        

        with open("/home/pyj2001/f1tenth_car_sim/catkin_ws/src/reactive_methods/src/f1tenth_data.txt", "w") as f:
    
            json.dump(datas, f)
            

    def print_ranges(self, ranges):
        interval_len = len(ranges) / 15
        for i in range(len(ranges)):
            if i % interval_len == 0:
                print("Line ", i ,  ": ", int(ranges[i]), ", ", end='')
                value_rounded_down = math.floor(ranges[i])
                for i in range(int(value_rounded_down)):
                    print("*", end='')
                print()

def main(args):
    rospy.init_node("Wrapper_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.05)
    rospy.spin()

if __name__ == '__main__':
    ct = 0
    ct1 = 0
    m = 20
    main(sys.argv)
