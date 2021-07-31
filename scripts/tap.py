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

        # nodes = []  # Or get all from ED?
        # edge_dicts = rospy.get_param('edges', [])  # type: List[Mapping]
        # for edge_dict in edge_dicts:
        #     edge = Edge()
        #     genpy.message.fill_message_args(edge, [edge_dict])  # Convert structure in nested dict to ROS msg
        #
        #     self.G.add_edge((edge.origin.entity, edge.origin.area),
        #                     (edge.destination.entity, edge.destination.area),
        #                     weight=edge.cost,
        #                     action=edge.action)

        self.__generate_dummy_graph()

    def _srv_plan_cb(self, req):
        # type: (PlanRequest) -> PlanResponse

        try:
            path = nx.shortest_path(self.G,
                                    (req.origin.entity, req.origin.area),
                                    (req.destination.entity, req.destination.area))

            edges = []
            for u, v in zip(path, path[1:]):
                edges += [Edge(origin=Node(*u),
                               destination=Node(*v),
                               cost=self.G[u][v]['weight'],
                               action_type=self.G[u][v]['action'])]
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
                            action=random.choice([Edge.ACTION_DRIVE]*6 +
                                                 [Edge.ACTION_OPEN_DOOR]*3 +
                                                 [Edge.ACTION_DRIVE]))


if __name__ == "__main__":
    rospy.init_node("topological_action_planner")

    tap = TopologicalActionPlanner()

    # import matplotlib.pyplot as plt
    # nx.draw(tap.G, with_labels=True)
    # plt.show()

    rospy.spin()
