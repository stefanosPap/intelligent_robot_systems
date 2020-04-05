#!/usr/bin/env python

import rospy
import math
import time

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation

# Class for assigning the robot speeds 
class RobotController:

    # Constructor
    def __init__(self):
        
      # Debugging purposes
      self.print_velocities = rospy.get_param('print_velocities')

      # Where and when should you use this?
      self.stop_robot = False

      # Create the needed objects
      self.sonar_aggregation = SonarDataAggregator()
      self.laser_aggregation = LaserDataAggregator()
      self.navigation  = Navigation()

      self.linear_velocity  = 0
      self.angular_velocity = 0

      # Check if the robot moves with target or just wanders
      self.move_with_target = rospy.get_param("calculate_target")

      # The timer produces events for sending the speeds every 110 ms
      rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
      self.velocity_publisher = rospy.Publisher(\
              rospy.get_param('speeds_pub_topic'), Twist,\
              queue_size = 10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):
        
      # Produce speeds
      self.produceSpeeds()

      # Create the commands message
      twist = Twist()
      twist.linear.x = self.linear_velocity
      twist.linear.y = 0
      twist.linear.z = 0
      twist.angular.x = 0 
      twist.angular.y = 0
      twist.angular.z = self.angular_velocity

      # Send the command
      self.velocity_publisher.publish(twist)

      # Print the speeds for debuggind purposes
      if self.print_velocities == True:
        print "[L,R] = [" + str(twist.linear.x) + " , " + \
            str(twist.angular.z) + "]"

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
      scan = self.laser_aggregation.laser_scan
      linear  = 0
      angular = 0
      ############################### NOTE QUESTION ############################
      # Check what laser_scan contains and create linear and angular speeds
      # for obstacle avoidance
      min_range = min(scan)
      min_range_angle = scan.index(min(scan))
    
      max_range = max(scan)
      max_range_angle = scan.index(max(scan))
       
      first_low_boundary = 0
      first_up_boundary = len(scan)
      second_low_boundary = -2.094 #minimum angle of a laser scan message 
      second_up_boundary = 2.094 #maximum angle of a laser scan message

      #map function to convert the indexes of scan array to the range of the real angles of the laser range 
      #convert (0  -  scan's length) -> (-2.094  -  2.094)
      map_fun = lambda x: (x - first_low_boundary)*(second_up_boundary - second_low_boundary) / (first_up_boundary - first_low_boundary) + second_low_boundary 
      
      min_range_angle = map_fun(min_range_angle)
      max_range_angle = map_fun(max_range_angle)  
     
      #negative angular turns the robot right and positive left
      
      #speed's values are assigned in 0 - 100 range and at the end of the calculations they will be converted in the range -0.3  -  0.3
      #using the map_fun_percent function
      #-100 -> -0.3 , 0 -> 0 , 100 -> 0.3
      angular = 0
      linear = 60
      #speed's weights are changing according to some criteria 
      
      #1st criterion 
      count_right = 0
      count_left = 0
      for i in range(len(scan)):
          if (scan[i] == max_range or scan[i] >= 0.7 * max_range):
            if i < len(scan)/2:
              count_right += 1 
            else:
              count_left += 1
              
      if count_right > count_left:
        angular += -28
      else:
        angular += 28
      #2nd criterion
      if min_range < 1:
        if min_range_angle < 0:
          angular += 22
       
        else:
          angular += -22
          
       #3rd criterion 
      if min_range < 0.2:
        if min_range_angle < 0:
          angular += 30
          linear = 10
        else:
          angular += -30
          linear = 10
          
      #4th criterion 
      count_crash_right=0
      count_crash_left=0

      for i in range(200,300):
        if scan[i] < 0.4:
          count_crash_right +=1

      for i in range(300,400):
        if scan[i] < 0.4:
          count_crash_left +=1
      
      if (count_crash_left + count_crash_right) > 60:
        if count_crash_left > count_crash_right:
          angular += -20
          linear = 0
        else:
          angular += 20
          linear = 0
      
      first_low_boundary = -100 
      first_up_boundary = 100
      second_low_boundary = -0.3 #minimum angle of a laser scan message 
      second_up_boundary = 0.3 #maximum angle of a laser scan message
      map_fun_percent = lambda x: (x - first_low_boundary)*(second_up_boundary - second_low_boundary) / (first_up_boundary - first_low_boundary) + second_low_boundary       
      
      angular = map_fun_percent(angular)
      linear = map_fun_percent(linear)
      ##########################################################################
      
      return [linear, angular]
      ##########################################################################
      return [linear, angular]

    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):
 
      # Produce target if not existent
      if self.move_with_target == True and \
              self.navigation.target_exists == False:

        # Create the commands message
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        # Send the command
        self.velocity_publisher.publish(twist)
        self.navigation.selectTarget()

      # Get the submodule's speeds
      [l_laser, a_laser] = self.produceSpeedsLaser()
      
      # You must fill these
      self.linear_velocity  = 0
      self.angular_velocity = 0
      
      if self.move_with_target == True:
        [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()
        ############################### NOTE QUESTION ############################
        # You must combine the two sets of speeds. You can use motor schema,
        # subsumption of whatever suits your better.

        ##########################################################################
      else:
        ############################### NOTE QUESTION ############################
        # Implement obstacle avoidance here using the laser speeds.
        # Hint: Subtract them from something constant
        self.linear_velocity  = l_laser
        self.angular_velocity = a_laser        
        ##########################################################################

    # Assistive functions
    def stopRobot(self):
      self.stop_robot = True

    def resumeRobot(self):
      self.stop_robot = False
