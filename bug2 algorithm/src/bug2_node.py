#!/usr/bin/python

import rospy, sys
import time

from bug2 import Bug2

if __name__ == "__main__":
    rospy.init_node('bug2', argv=sys.argv)
    x_target = rospy.get_param('~goal_x')
    y_target = rospy.get_param('~goal_y')
    bug = Bug2(x_target, y_target)
    time.sleep(10)
    bug.run()