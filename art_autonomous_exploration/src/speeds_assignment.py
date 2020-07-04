#!/usr/bin/env python3

import rospy
import math
import time
import numpy as np 
from numpy import interp 
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
      scan = self.laser_aggregation.laser_scan                    # the message is of type sensor_msgs/LaserScan 
      angle_increment = self.laser_aggregation.angle_increment    # we assign its attributes to local variables in order to use them in the method 
      range_min = self.laser_aggregation.range_min
      range_max = self.laser_aggregation.range_max
      angle_max = self.laser_aggregation.angle_max
      angle_min = self.laser_aggregation.angle_min
      header = self.laser_aggregation.header 
      ############################### NOTE QUESTION ############################
      # Check what laser_scan contains and create linear and angular speeds
      # for obstacle avoidance
      '''
      #1st criterion 
      turn_right = 0
      turn_left = 0
      for i in range(len(scan)):
          if (scan[i] == max_range or scan[i] >= 0.7 * max_range):
            if i < len(scan)/2:
              turn_right += 1 
            else:
              turn_left += 1
              
      if turn_right > turn_left:
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
      '''                                   
      map_fun = lambda x: angle_min + x * angle_increment # convert (0  -  scan's length) -> (-2.094  -  2.094)  
                                                          # negative angular turns the robot right and positive left 
                                                          # initialize linear and angular velocities to zero 
      linear_summary = 0
      angular_summary = 0
      
      for i in range(270,430):                          # for loop for calclulating linear velocity. We take into consideration mostly scanlines in range 270 - 430  
        angle = map_fun(i)                              # because this is the front view of the robot. This range calculated after experiments. 
        linear_summary += math.cos(angle) * scan[i]     # Linear velocity is calculated through the summary of cosine(angle) * scan[i] for every scan. Cosine is needed for  
                                                        # the sign of every element that is part of the summary (angles in range -90 -> 90 give positive sign and angles in range 90 -> 270 give negative sign)
                                                        # and the scan[i] is needed for the measure of the calculation. The measurement is proportional to scan[i]. 
      
      for i in range(200,500):                          # this for loop is needed in order to avoid collisions when the robot has its front view near an obstacle. Therefore we use the range   
        if scan[i] < 0.2:                               # 200 -> 500 which is calculated after experiments. When this case happens we set linear velocity equal to zero. 
          linear_summary = 0
          break
        
      for i in range(0,300):                            # These for loops are used for calclulating angular velocity. We take into consideration mostly scanlines in range 0 - 300
        angle = map_fun(i)                              # and 400 - 667 because these are the left and right sides of the robot. These ranges calculated after experiments.   
        angular_summary += math.sin(angle) * (scan[i])  # Angular velocity is calculated through the summary of sin(angle) * scan[i] for every scan. Sin is needed for the sign of every element  
                                                        # that is part of the summary (angles in range 0 -> 180 give positive sign and angles in range 180 -> 360 give negative sign)
                                                        # and the scan[i] is needed for the measure of the calculation. The measurement is proportional to scan[i]. 
      for i in range(400,len(scan)):
        angle = map_fun(i)  
        angular_summary += math.sin(angle) * (scan[i])
      
      if linear_summary == 0:                           # in the case that the robot stucks in front of an obstacle because of zero linear speeed, we increase the angular velocity in order    
        angular_summary = angular_summary * 10          # to turn faster and leave the narrow area.  
          
      linear =  math.tanh(linear_summary / 1000) * 0.3  # apply tanh mathematical function and multiply with 0.3 in order to obtain a value in range -0.3 -> 0.3  
      angular = math.tanh(angular_summary / 1000) * 0.3 
      
      return [linear, angular]
      ##########################################################################
      

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

        if l_laser != 0:
            parameter_1 = 0.8
            parameter_2 = 0.2 
            self.linear_velocity = (parameter_1 * l_goal + parameter_2 * l_laser) / (parameter_2 + parameter_1)
            self.angular_velocity = (parameter_1 * a_goal + parameter_2 * a_laser) / (parameter_2 + parameter_1) 
        else:
            parameter_1 = 0
            parameter_2 = 1 
            self.linear_velocity = (parameter_1 * l_goal + parameter_2 * l_laser) / (parameter_2 + parameter_1)
            self.angular_velocity = (parameter_1 * a_goal + parameter_2 * a_laser) / (parameter_2 + parameter_1) 
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
