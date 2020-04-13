#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

# Class for reading the data from the laser sensor
class LaserDataAggregator:

    # Constructor
    def __init__(self):

        # Initialization of laser scan 
        self.laser_scan = []
        self.angle_increment = 0
        self.range_min = 0
        self.range_max = 0
        self.angle_min = 0
        self.angle_max = 0
        self.header = None
        # ROS Subscribers to the robot's laser
        laser_topic = rospy.get_param("laser_topic")
        rospy.Subscriber(laser_topic, LaserScan, self.getDataLaser) 

    # Getting data from the laser
    def getDataLaser(self, data):

        # Get the measurements
        self.laser_scan = list(data.ranges)
        self.angle_increment = data.angle_increment
        self.range_max = data.range_max
        self.range_min = data.range_min
        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.header = data.header 
        # Pay attention for special values
        for i in range(0, len(self.laser_scan)):
            if self.laser_scan[i] > data.range_max:
                self.laser_scan[i] = data.range_max
            elif self.laser_scan[i] < data.range_min:
                self.laser_scan[i] = data.range_min

