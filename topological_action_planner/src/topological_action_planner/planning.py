import math

import rospy
import networkx as nx
from typing import List, Mapping, Optional, Tuple

from topological_action_planner_msgs.msg import Edge, Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import tf2_ros

from ed_py.world_model import WM
from cb_base_navigation_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from ed_navigation_msgs.srv import GetGoalConstraint
from cb_base_navigation_msgs.msg import PositionConstraint


def compute_path_length(path: List[PoseStamped]) -> float:
    """
    Calculate the total length of a path by summing up the lengths of all edges

    :param path: List of PoseStampeds (assumed to be all in the same frame)
    :return sum of cartesian distances between given PoseStamped
    """
    return sum(
        math.hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y)
        for a, b in zip(path, path[1:])
    )


class EdgeCostCalcBase:
    def __call__(self, edge: Edge) -> Optional[float]:
        raise NotImplemented()


class EdgeCostCalc(EdgeCostCalcBase):
    """The EdgeCostCalc calculates a cost for an Edge"""
    def __init__(self, wm: WM, default_action_costs: Mapping[str, int]):
        self.wm = wm
        self._action_costs = default_action_costs

        self._get_constraint_srv = rospy.ServiceProxy("ed/navigation/get_constraint", GetGoalConstraint)
        self._global_planner = rospy.ServiceProxy("global_planner/get_plan_srv", GetPlan)

    def _get_area_constraint(self, entity: str, area: str) -> PositionConstraint:
        """What is the center pose of an entity and area?"""
        res = self._get_constraint_srv(entity_ids=[entity], area_names=[area])
        if not res.error_msg:
            return PositionConstraint(constraint=res.position_constraint_map_frame, frame="map")
        else:
            raise Exception("Cannot get a constraint for {}.{}".format(entity, area))

    def __call__(self, edge: Edge) -> Optional[float]:
        # TODO: we should actually plan from the end of the plan found for the previous edge.
        # Otherwise the center pose could be blocked but not e whole area and we would still fail.
        entity = self.wm.get_entity(edge.origin.entity)
        if not entity:
            rospy.logwarn("No entity '{}'".format(edge.origin.entity))
            return None

        if edge.origin.area:
            src_vector = entity.volumes[edge.origin.area].center_point
            src = PoseStamped(
                header=Header(frame_id=entity.uuid),
                pose=Pose(
                    position=Point(src_vector.x(), src_vector.y(), 0),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
            )
        else:  # In case we have eg. robot as the source
            src = tf2_ros.convert(entity.pose, PoseStamped)

        dst = self._get_area_constraint(edge.destination.entity, edge.destination.area)
        global_plan_res = self._global_planner(
            GetPlanRequest(start=src, goal_position_constraints=[dst])
        )

        if global_plan_res.succes:
            edge_cost = (
                    compute_path_length(global_plan_res.plan) * self._action_costs[Edge.ACTION_DRIVE]
            )
            # edge_cost = 100 * 0.1 * self._action_costs[Edge.ACTION_DRIVE]
        else:
            # Cannot plan along this edge
            edge_cost = 100

        rospy.loginfo(
            "Updating cost of edge {} - {} = {}".format(
                edge.origin, edge.destination, edge_cost
            ).replace("\n", ", ")
        )
        return edge_cost


class TopoPlanner:
    def __init__(self, edge_cost_calc: EdgeCostCalcBase):
        """
        edge_cost_calc is a separate class so that the TopoPlanner itself can be tested separately
        """
        self._edge_cost_calc = edge_cost_calc

    def plan(self, graph: nx.Graph, origin_node: Tuple[str, str], dst_node: Tuple[str, str]) -> List[Edge]:
        lowest_total_cost = float("inf")
        while True:
            path = nx.shortest_path(graph, origin_node, dst_node, weight="weight")

            edges = []
            for u, v in zip(path, path[1:]):
                edges += [
                    Edge(
                        origin=Node(*u),
                        destination=Node(*v),
                        cost=graph[u][v]["weight"],
                        action_type=graph[u][v]["action_type"],
                    )
                ]

            # For the drive edges, query the maybe now updated cost of driving that with current knowledge
            for edge in edges:
                if edge.action_type == Edge.ACTION_DRIVE:
                    edge_cost = self._edge_cost_calc(edge)
                    if edge_cost is None:
                        continue
                    graph[(edge.origin.entity, edge.origin.area)][(edge.destination.entity, edge.destination.area)][
                        "weight"
                    ] = edge_cost
                    edge.cost = edge_cost

            current_total_cost = sum([edge.cost for edge in edges])
            if lowest_total_cost == current_total_cost:
                rospy.loginfo("The current plan has the lowest cost ({}), picking this".format(current_total_cost))
                break  # The cost is not changing anymore, we hit the optimum, use this path
            elif current_total_cost < lowest_total_cost:
                rospy.loginfo(
                    "The current plan has cost {}, lowest is {}. "
                    "Trying a to find a better plan with updated edge costs".format(
                        current_total_cost, lowest_total_cost
                    )
                )
                lowest_total_cost = current_total_cost
        return edges

