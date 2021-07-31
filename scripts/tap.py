#! /usr/bin/env python

import rospy
import genpy
import networkx as nx

from typing import List, Mapping

from topological_action_planner.msg import Edge, Node
from topological_action_planner.srv import Plan, PlanRequest, PlanResponse
from topological_action_planner.srv import UpdateEdge, UpdateEdgeRequest, UpdateEdgeResponse


class TopologicalActionPlanner:
    def __init__(self):
        self.G = nx.Graph()

        self._srv_plan = rospy.Service('~get_plan', Plan, self._srv_plan_cb)
        self._srv_update_edge = rospy.Service('~update_edge', Plan, self._srv_update_edge_cb)

        nodes = []  # Or get all from ED?
        edge_dicts = rospy.get_param('edges', [])  # type: List[Mapping]
        for edge_dict in edge_dicts:
            edge = Edge()
            genpy.message.fill_message_args(edge, [edge_dict])  # Convert structure in nested dict to ROS msg

            self.G.add_edge((edge.u.entity, edge.u.area),
                            (edge.v.entity, edge.v.area),
                            weight=edge.cost,
                            action=edge.action)

    def _srv_plan_cb(self, req):
        # type: (PlanRequest) -> PlanResponse
        # path = nx.shortest_path(self.G,
        #                         (req.origin.entity, req.origin.area),
        #                         (req.destination.entity, req.destination.area))

        a = Node(req.origin.entity, req.origin.area)
        b = Node('inner_door', 'in_front_of')
        c = Node('inner_door', 'in_back_of')
        d = Node(req.destination.entity, req.destination.area)
        edge_ab = Edge(u=a, v=b, cost=1, action_type=Edge.ACTION_DRIVE)
        edge_bc = Edge(u=b, v=c, cost=10, action_type=Edge.ACTION_OPEN_DOOR)
        edge_cd = Edge(u=c, v=d, cost=1, action_type=Edge.ACTION_DRIVE)

        return PlanResponse(success=True,
                            edges=[edge_ab, edge_bc, edge_cd])

    def _srv_update_edge_cb(self):
        # type: (UpdateEdgeRequest) -> UpdateEdgeResponse
        pass


if __name__ == "__main__":
    rospy.init_node("topological_action_planner")

    tap = TopologicalActionPlanner()

    rospy.spin()
