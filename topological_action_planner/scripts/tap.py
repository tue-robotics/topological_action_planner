#! /usr/bin/env python
import copy
import math

import rospy
import networkx as nx
from typing import List

from topological_action_planner_msgs.msg import Edge, Node
from topological_action_planner_msgs.srv import Plan, PlanRequest, PlanResponse
from topological_action_planner_msgs.srv import (
    UpdateEdge,
    UpdateEdgeRequest,
    UpdateEdgeResponse,
)
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import tf2_ros
import pykdl_ros

from ed_py.utility import rooms_of_volume, rooms_of_entity
from ed_py.world_model import WM
from topological_action_planner.serialisation import from_dicts
from topological_action_planner.planning import TopoPlanner
from topological_action_planner.util import visualize, generate_dummy_graph
from topological_action_planner.visualisation import create_tap_marker_array
from cb_base_navigation_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from ed_navigation_msgs.srv import GetGoalConstraint
from cb_base_navigation_msgs.msg import PositionConstraint


class TopoPlannerNode:
    def __init__(self):
        self.G = from_dicts(rospy.get_param("~edges"))
        self.plot = rospy.get_param("~plot", False)

        # TODO: These are magic numbers. Can be parametrized, but these are the defaults
        action_costs = rospy.get_param(
            "~action_costs",
            {
                Edge.ACTION_DRIVE: 1,  # per meter distance driven
                Edge.ACTION_OPEN_DOOR: 5,  # per door opened
                Edge.ACTION_PUSH_OBJECT: 10,  # per item pushed
            },
        )

        self._wm = WM()
        self._topo_planner = TopoPlanner(self._wm, action_costs)

        self._get_constraint_srv = rospy.ServiceProxy("ed/navigation/get_constraint", GetGoalConstraint)

        self._srv_plan = rospy.Service("~get_plan", Plan, self._srv_plan_cb)
        self._srv_update_edge = rospy.Service("~update_edge", UpdateEdge, self._srv_update_edge_cb)
        self._pub_grasp_marker = rospy.Publisher("~graph", MarkerArray, queue_size=10, latch=True)

        # self.G = generate_dummy_graph()
        # to_yaml(self.G, '/tmp/graph.yaml')
        if self.plot:
            visualize(self.G)
        self._pub_grasp_marker.publish(create_tap_marker_array(self.G, self._wm))

        rospy.loginfo("Topological action planner started")

    def _srv_plan_cb(self, req: PlanRequest) -> PlanResponse:
        """
        Query the graph for a plan from an origin to some destination.
        If the origin is not given, interpret that as starting from the robot's current position.
        """
        graph = copy.deepcopy(self.G)  # type: nx.Graph

        for node in graph.nodes.keys():
            node0_entity = self._wm.get_entity(node[0])
            graph.nodes[node]["room"] = rooms_of_volume(
                self._wm, node0_entity, node[1]
            )  # Based on where they are in ED?

        if req.origin.entity == "":
            # This indicates the plan starts from the robot's current pose
            # We'll insert a node corresponding to our current position.
            # Difficulty is determining it's adjacency to other nodes, based on geometry
            robot_name = rospy.get_namespace().split("/")[-2]
            robot_entity = self._wm.get_entity(robot_name)
            origin_node = robot_name, ""
            # Ask ED in which room the robot is currently, maybe based on it's pose
            current_rooms = rooms_of_entity(self._wm, robot_entity)
            current_room_ids = [r.uuid for r in current_rooms]
            # TODO: This assumes that the robot can always drive to any other node in the same room.
            # This might not always be true of course. It may also make more sense to only connect with the closest N waypoints
            nodes_in_same_room = [
                node for node, info in graph.nodes.items() if info["room"][0].uuid in current_room_ids
            ]
            if not nodes_in_same_room:
                # TODO: What happens when the robot is not in a defined room? But eg. just outside the apartment
                rospy.logerr("There are no nodes to connect the robot to")
            for node in nodes_in_same_room:
                # TODO: Will be updated later when taking path length into account later
                graph.add_edge(origin_node, node, action_type=Edge.ACTION_DRIVE, weight=1)
        else:
            origin_node = req.origin.entity, req.origin.area

        dst_node = req.destination.entity, req.destination.area
        rospy.loginfo("Requesting topological action plan from {} to {}".format(origin_node, dst_node))

        try:
            edges = self._topo_planner.plan(graph, origin_node, dst_node)

            self._pub_grasp_marker.publish(create_tap_marker_array(graph, self._wm))
            rospy.loginfo("Found plan of {} edges".format(len(edges)))
            return PlanResponse(error_msg="", error_code=PlanResponse.SUCCESS, edges=edges)
        except nx.NetworkXNoPath as no_path_found_ex:
            rospy.logerr(no_path_found_ex)
            return PlanResponse(
                error_msg=str(no_path_found_ex),
                error_code=PlanResponse.ERROR_NO_PATH_FOUND,
            )
        except nx.NodeNotFound as node_not_found_ex:
            rospy.logerr(node_not_found_ex)
            return PlanResponse(
                error_msg=str(node_not_found_ex),
                error_code=PlanResponse.ERROR_UNKNOWN_NODE,
            )

    def _srv_update_edge_cb(self, req: UpdateEdgeRequest) -> UpdateEdgeResponse:
        try:
            self.update_edge(req.updated)
            return UpdateEdgeResponse(success=True)
        except nx.NodeNotFound as node_not_found_ex:
            rospy.logerr(node_not_found_ex)
            return UpdateEdgeResponse(success=False)

    def update_edge(self, edge: Edge):
        """
        Update the cost and action_type of an edge
        """
        u, v = (edge.origin.entity, edge.origin.area), (
            edge.destination.entity,
            edge.destination.area,
        )
        self.G[u][v]["weight"] = edge.cost
        self.G[u][v]["action_type"] = edge.action_type


if __name__ == "__main__":
    rospy.init_node("topological_action_planner")

    tap = TopoPlannerNode()

    rospy.spin()
