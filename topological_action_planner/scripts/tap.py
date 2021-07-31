#! /usr/bin/env python
import os

import yaml

import rospy
import genpy
import networkx as nx

from typing import List, Mapping

from topological_action_planner_msgs.msg import Edge, Node
from topological_action_planner_msgs.srv import Plan, PlanRequest, PlanResponse
from topological_action_planner_msgs.srv import UpdateEdge, UpdateEdgeRequest, UpdateEdgeResponse

from topological_action_planner.serialisation import from_dicts


class TopologicalActionPlanner:
    def __init__(self):
        self.G = from_dicts(rospy.get_param('~edges'))

        self._srv_plan = rospy.Service('~get_plan', Plan, self._srv_plan_cb)
        self._srv_update_edge = rospy.Service('~update_edge', Plan, self._srv_update_edge_cb)

        # self.__generate_dummy_graph()
        # to_yaml(self.G, '/tmp/graph.yaml')

        import matplotlib.pyplot as plt
        nx.draw(self.G, with_labels=True)
        plt.ion()
        plt.show()
        plt.pause(0.001)

    def _srv_plan_cb(self, req):
        # type: (PlanRequest) -> PlanRespons

        try:
            path = nx.shortest_path(self.G,
                                    (req.origin.entity, req.origin.area),
                                    (req.destination.entity, req.destination.area))

            edges = []
            for u, v in zip(path, path[1:]):
                edges += [Edge(origin=Node(*u),
                               destination=Node(*v),
                               cost=self.G[u][v]['weight'],
                               action_type=self.G[u][v]['action_type'])]
            # TODO: for the drive edges, query the maybe now updated cost of driving that with current knowledge

            return PlanResponse(error_msg='', error_code=PlanResponse.SUCCESS, edges=edges)
        except nx.NetworkXNoPath as no_path_found_ex:
            rospy.logerr(no_path_found_ex)
            return PlanResponse(error_msg=str(no_path_found_ex), error_code=PlanResponse.ERROR_NO_PATH_FOUND)
        except nx.NodeNotFound as node_not_found_ex:
            rospy.logerr(node_not_found_ex)
            return PlanResponse(error_msg=str(node_not_found_ex), error_code=PlanResponse.ERROR_UNKNOWN_NODE)

    def _srv_update_edge_cb(self):
        # type: (UpdateEdgeRequest) -> UpdateEdgeResponse
        pass

    def __generate_dummy_graph(self, n_nodes=10, n_edges=20):
        numbers = '0123456789'
        letters = 'abcdefghijklmnopqrstuvwxyz'
        import random
        random.seed(123456789)
        # Make a bunch of nodes with randomly picked names from numbers and letters
        nodes = [(random.choice(numbers), random.choice(letters)) for _ in range(n_nodes)]

        for _ in range(n_edges):
            a, b = random.choice(nodes), random.choice(nodes)
            if a == b:
                continue
            self.G.add_edge(a,
                            b,
                            weight=random.random(),
                            # 60% chance of driving, 30% doors, 10% pushing:
                            action_type=random.choice([Edge.ACTION_DRIVE]*6 +
                                                      [Edge.ACTION_OPEN_DOOR]*3 +
                                                      [Edge.ACTION_DRIVE]))


if __name__ == "__main__":
    rospy.init_node("topological_action_planner")

    tap = TopologicalActionPlanner()

    rospy.spin()
