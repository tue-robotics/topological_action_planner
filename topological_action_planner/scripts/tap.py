#! /usr/bin/env python

import rospy
from topological_action_planner.node import TopoPlannerNode

if __name__ == "__main__":
    rospy.init_node("topological_action_planner")

    tap = TopoPlannerNode()

    rospy.spin()
