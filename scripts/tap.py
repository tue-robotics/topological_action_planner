#! /usr/bin/env python

import rospy
import networkx as nx

from tap.srv import Plan, PlanRequest, PlanResponse


class TopologicalActionPlanner:
    def __init__(self):
        self.G = nx.Graph()

    def _srv_plan_cb(self, req):
        # type: (PlanRequest) -> PlanReponse


if __name__ == "__main__":
    rospy.init_node("topological_action_planner")

    tap = TopologicalActionPlanner()

    rospy.spin()
