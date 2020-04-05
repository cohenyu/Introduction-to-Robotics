#!/usr/bin/python

import math
import sys
import time
import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import platform


class Direction:
    LEFT = 1
    RIGHT = -1

class Mode:
    MLINE = 0
    FORWARD = 1
    OBSTACLE = 2
    WALL = 3
    CORNER = 4

class Bug2(object):

    def __init__(self, x_target, y_target):
        # get the target point
        self.x_target = x_target
        self.y_target = y_target

        self.m = 0
        self.b = 0

        self.range = 1
        self.min_dist_from_obstacle = 1
        self.rate = rospy.Rate(10)
        self.forward_speed = 0.4
        self.rotation_speed = 0.3
        self.not_finish = True

        self.mode = Mode.FORWARD
        self.free_wall = True
        self.check_mline = False
        self.corner_move = 0

        self.command_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber("scan", LaserScan, self.scan_callback, queue_size=1)

    # this function runs the algorithm
    def run(self):
        self.setup()
        self.rotate_to_goal(True)
        while not rospy.is_shutdown():

            # if the robot is on the mline- finish and then go to the goal
            if self.mode == Mode.MLINE:
                break

            # if the robot is in forward mode - go on the mline to the goal
            elif self.mode == Mode.FORWARD:
                self.move(self.forward_speed, 0)
                if self.is_finished():
                    self.not_finish = False
                    break

            # if the robot is in obstacle mode - rotate to left
            elif self.mode == Mode.OBSTACLE:
                self.move(0, self.rotation_speed, Direction.LEFT)

            #  if the robot is in wall mode - move forward or left
            elif self.mode == Mode.WALL:
                self.corner_move = 0
                if self.free_wall:
                    self.move(self.forward_speed, 0)
                else:
                    self.move(0, self.rotation_speed, Direction.LEFT)

            #  if the robot is in corner mode - do more little step and then rotate right
            elif self.mode == Mode.CORNER:
                self.check_mline = True

                # 10 steps forward
                if self.corner_move < 10:
                    self.corner_move = self.corner_move + 1
                    self.move(self.forward_speed, 0)

                # rotate to right
                if self.corner_move >= 10:
                    self.move(0, self.rotation_speed, Direction.RIGHT)

            # if we need to check the mline, check and change to mline mode
            if self.check_mline:
                if self.is_mline_mode():
                    self.mode = Mode.MLINE

            self.rate.sleep()

        # if we not on the goal point - move to the goal
        if self.not_finish:
            self.rotate_to_goal(False)
            while not rospy.is_shutdown():
                self.move(self.forward_speed, 0)
                if self.is_finished():
                    break
                self.rate.sleep()

    # This function check if there is obstacle in front of the robot
    def is_obstacle(self, ranges):
        for dist in ranges:
            if dist < self.min_dist_from_obstacle:
                self.mode = Mode.OBSTACLE
                return True

    #  this function checks if the robot need to be in the mline mode
    def is_mline_mode(self):
        x, y, rot = self.get_cur_pos()
        return self.is_mline(x, y)


    # This function is the setup of the program, (define the mline)
    def setup(self):
        cur_x, cur_y, rot = self.get_cur_pos()
        self.define_mline(self.x_target, self.y_target, cur_x, cur_y)

    #  This is the callback function
    def scan_callback(self, scan_msg):
        #  check if there is no obstacle on the wall or the robot no too close to the wall
        self.free_wall = scan_msg.ranges[0] >= self.min_dist_from_obstacle

        # if it is in forward mode and there is obstacle
        if self.mode == Mode.FORWARD and self.is_obstacle(scan_msg.ranges):
            # change to obstacle mode
            self.mode = Mode.OBSTACLE

        # if the mode is obstacle and the middle laser is free
        elif self.mode == Mode.OBSTACLE and math.isnan(scan_msg.ranges[int(len(scan_msg.ranges) / 2)]):
            # change to wall mode
            self.mode = Mode.WALL

        # if the mode is wall and the right laser is free
        elif self.mode == Mode.WALL and math.isnan(scan_msg.ranges[0]):
            # change to corner mode
            self.mode = Mode.CORNER

        # if the mode is corner and the right laser is free
        elif self.mode == Mode.CORNER and not math.isnan(scan_msg.ranges[0]):
            # change to wall mode
            self.mode = Mode.WALL

        self.rate.sleep()

    # This function calculate the diff
    def get_diff(self, step):
        curr_x, curr_y, curr_angle = self.get_cur_pos()
        return step - curr_angle


    # This function publish a move msg according to the direction and the speeds
    def move(self, x, z, direction = 0):
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = direction * z
        self.command_pub.publish(msg)

    #  This function calculate the movement
    def movement(self, x, y):
        angular = 4 * math.atan2(y, x)
        linear = 0.5 * math.sqrt(x ** 2 + y ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

    # This function calculate the slope of the mline
    def get_slope(self, x1, y1, x2, y2):
        if (x1 - x2) == 0:
            return  math.inf
        return (y1 - y2) / (x1 - x2)


    # This function define the m and b of the mline
    def define_mline(self, x1, y1, x2, y2):
        self.m = self.get_slope(x1, y1, x2, y2)
        self.b = x1 if math.isinf(self.m) else y1 - (x1 * self.m)

    # This function checks if the robot is on the mline
    def is_mline(self, x, y):
        if math.isinf(self.m):
            return math.fabs(x - self.goal_x) < 1
        else:
            new_y =  (x * self.m) + self.b
            return math.fabs(new_y - y) < 1

    # This function rotates the robot to the goal direction
    def rotate_to_goal(self, start):
        curr_x, curr_y, angle = self.get_cur_pos()
        angular = math.atan2((self.y_target - curr_y), (self.x_target - curr_x))
        if not start:
            angle = 0
        to_move = angular + angle
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0.5
        curr_x, curr_y, curr_angle = self.get_cur_pos()
        diff = to_move - curr_angle
        while not (-0.06 < diff < 0.06):
            self.command_pub.publish(twist_msg)
            curr_x, curr_y, curr_angle = self.get_cur_pos()
            diff = to_move - curr_angle

    # This function get the current position of the robot
    def get_cur_pos(self):
        listener = tf.TransformListener()
        listener.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(10.0))
        try:
            (trans, rot) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            robot_x = trans[0]
            robot_y = trans[1]
            (roll, pitch, angle) = tf.transformations.euler_from_quaternion(rot)
            return robot_x, robot_y, angle
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Service call failed: %s" % e)

    # This function check if the robot is in the goal point
    def is_finished(self):
        x_pos, y_pos, rot = self.get_cur_pos()
        return math.fabs(self.x_target - x_pos) < self.range and math.fabs(self.y_target - y_pos) < self.range


    # This function check the direction that the robot need to rotate
    def check_direction(self, scan_msg):
        half_ranges_len = len(scan_msg.ranges) / 2
        left_nans = 0
        left_dist = 0
        right_nans = 0
        right_dist = 0

        # loop over the ranges array from the sensor
        for idx, dist in enumerate(scan_msg.ranges):
            # calculate the number of nans in each side of the robot
            if math.isnan(dist):
                if idx <= half_ranges_len:
                    right_nans += 1
                else:
                    left_nans += 1
            # if it is not nan - there is an obstacle
            else:
                # calculate the distance for each side of the robot
                if idx <= half_ranges_len:
                    right_dist += dist
                else:
                    left_dist += dist

        # if there are more nans from the right- go right
        if right_nans > left_nans:
            direction = -1
        # if there are more nans from the left- go left
        elif left_nans > right_nans:
            direction = 1
        # in case the number of nans are equal-
        else:
            # if the left distance is bigger then right - go left
            if left_dist > right_dist:
                direction = 1
            # if the right distance is bigger then left - go right
            else:
                direction = -1
        return direction

