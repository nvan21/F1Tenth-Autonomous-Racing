#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math
        
class FollowTheGap:
    def __init__(self):
        rospy.init_node("follow_the_gap")

        # Get topic names
        drive_topic = rospy.get_param("~gap_drive_topic", "/drive")
        scan_topic = rospy.get_param("~scan_topic", "/scan")

        # Make a publisher for drive messages
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

        # Start a subscriber to listen to laser scan messages
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.lidar_callback)

        # Get physical system constraints
        self.max_speed = rospy.get_param("~max_speed")
        self.max_steering_angle = rospy.get_param("~max_steering_angle")
        self.max_accel = rospy.get_param("~max_accel")
        self.max_decel = rospy.get_param("~max_decel")
        self.max_steering_velocity = rospy.get_param("~max_steering_vel")
        self.car_width = rospy.get_param("~width")
        
        # Get LiDAR scan information
        self.scan_count = rospy.get_param("~scan_beams")
        self.scan_fov = rospy.get_param("~scan_field_of_view")
        self.scan_interval = self.scan_fov / self.scan_count 

        # Hyperparameters with the following definitions:
        # -----------------------------------------------
        # lidar_fov: The FOV that the robot will actually consider when finding the max gap
        # car_safety_radius: The safety radius of the car, in meters, that no lidar scan can be in when turning the car
        # consecutive_thresh: The distance, in meters, that counts as a discrepancy in the algorithm
        # consecutive_sample_thresh: The number of extra LiDAR scans that get changed when a discrepancy is detected
        # driving_mode: Decides whether the car drives extra quick or extra smooth (both modes are fast and smooth)
        # smooth_velocity_table: A piecewise step function that decides the speed the car should go based on distance in front of the car
        # fast_velocity_table: A piecewise step function that decides the speed the car should go based on distance in front of the car
        self.lidar_fov = np.radians(180)
        self.car_safety_radius = 0.25
        self.consecutive_thresh = 2.5
        self.consecutive_sample_thresh = 5
        self.driving_mode = "fast"
        self.smooth_velocity_table = {0 : self.max_speed / 3, 2: self.max_speed / 2, 5: self.max_speed / 1.5, 10: self.max_speed / 1}
        self.fast_velocity_table = {0 : self.max_speed / 2, 2: self.max_speed / 1.5, 5: self.max_speed / 1}

        # Separate positive LiDAR fov into two halves - a positive and negative one with true north = 0 degrees
        self.left_fov = self.lidar_fov / 2
        self.right_fov = -self.lidar_fov / 2

    # This method takes in the initial LiDAR scan and filters the data according to the algorithm. The methodology is explained in detail in the essay questions
    def preprocess_lidar(self, ranges):
        # Prune the LiDAR data to only keep the information that's within the specified FOV
        start_set = round(self.scan_count / 2 + self.right_fov / self.scan_interval)
        end_set = round(self.scan_count / 2 + self.left_fov / self.scan_interval)
        ranges = list(ranges[start_set:end_set + 1])

        # Initialize the variables that help track discrepancies
        last_r = 0
        skip_iter = False
        num_excluded_samples = 0

        # Loop through every range in the pruned LiDAR data
        for idx, r in enumerate(ranges):
            if idx == 0:
                last_r = r

            # If the algorithm just got done changing values due to a discrepancy, then skip the parent loop the same number of iterations that the discrepancy pruning loop did
            if skip_iter is True:
                last_r = r
                num_excluded_samples -= 1
                skip_iter = False if num_excluded_samples == -1 else True

                continue

            # Logic to decide whether a new value is considered a discrepancy
            if abs(r - last_r) > self.consecutive_thresh:
                # Calculate the number of excluded samples based on the angle between the car and the discrepancy and then create a range of the indexes that need changed
                num_excluded_samples = round(math.asin((self.car_width / 0.75 ) / min(last_r,r)) / self.scan_interval) + self.consecutive_sample_thresh
                excluded_samples = range(idx, idx + num_excluded_samples + 1) if r - last_r > 0 else range(idx - num_excluded_samples, idx)
                
                # Deals with excluded sample indexes being larger than the list has (happens if the discrepancy is near the left side of the FOV)
                if max(excluded_samples) >= len(ranges):
                    excluded_samples = range(idx, len(ranges) - 1)

                # Loop through all of the samples that need to be modified
                for sample_idx in excluded_samples:
                    # Logic to make sure that the excluded sample isn't being changed if it's already smaller than the new distance
                    if ranges[sample_idx] < min(r, last_r):
                        pass
                    else:
                        ranges[sample_idx] = min(r, last_r)

                skip_iter = True

            last_r = r

        return ranges

    def find_max_gap(self, ranges):
        max_gap = max(ranges)
        max_gap_idx = ranges.index(max_gap)

        return max_gap, max_gap_idx

    def lidar_callback(self, data):
        # Prepare the incoming LiDAR data for the algorithm
        ranges = self.preprocess_lidar(data.ranges)
        max_gap, max_gap_idx = self.find_max_gap(ranges)

        # Calculate the ideal steering angle based on the angle that the max gap was scanned at
        steering_angle = self.right_fov + max_gap_idx * self.scan_interval

        # If the ideal steering angle is larger than the maximum allowed steering angle, then set the steering angle to the max steering angle
        if abs(steering_angle) > self.max_steering_angle:
            steering_angle = steering_angle/abs(steering_angle) * self.max_steering_angle

        # Check the points from -90 to -135 and 90 to 135 degrees to make sure that they aren't within the specified car safety radius before turning
        # If they are, then give a slight turn away from the obstacle
        for r, l in zip(data.ranges[0:180], data.ranges[901:]):
            if l < self.car_safety_radius:
                steering_angle = -0.025
                break
            elif r < self.car_safety_radius:
                steering_angle = 0.025
                break

        # Get the velocity table based on the driving mode
        velocity_table = self.fast_velocity_table if self.driving_mode.lower() == "fast" else self.smooth_velocity_table

        # Loop through every step function in the piece-wise lookup table to find the max speed that the car can drive in its current pose
        for distance, velocity in velocity_table.items():
            if data.ranges[540] > distance:
                speed = velocity

        self.pub_drive(steering_angle, speed)
        
    def pub_drive(self, steering_angle, speed):
        #publish drive messages
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


if __name__ == "__main__":
    try:
        FollowTheGap()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
