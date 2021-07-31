#! /usr/bin/env python
import rospy
import networkx as nx

from topological_action_planner_msgs.msg import Edge, Node
from topological_action_planner_msgs.srv import Plan, PlanRequest, PlanResponse
from topological_action_planner_msgs.srv import UpdateEdge, UpdateEdgeRequest, UpdateEdgeResponse
from visualization_msgs.msg import MarkerArray

from topological_action_planner.serialisation import from_dicts
from topological_action_planner.util import visualize, generate_dummy_graph
from topological_action_planner.visualisation import create_tap_marker_array


class TopologicalActionPlanner:
    def __init__(self):
        self.G = from_dicts(rospy.get_param('~edges'))

        self._srv_plan = rospy.Service('~get_plan', Plan, self._srv_plan_cb)
        self._srv_update_edge = rospy.Service('~update_edge', Plan, self._srv_update_edge_cb)
        self._pub_grasp_marker = rospy.Publisher('~graph', MarkerArray, queue_size=10)

        # self.G = generate_dummy_graph()
        # to_yaml(self.G, '/tmp/graph.yaml')
        visualize(self.G)
        self._pub_grasp_marker.publish(create_tap_marker_array(self.G))

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


if __name__ == "__main__":
    rospy.init_node("topological_action_planner")

    tap = TopologicalActionPlanner()

    rospy.spin()
