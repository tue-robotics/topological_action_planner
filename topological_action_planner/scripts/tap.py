#! /usr/bin/env python
import copy

import rospy
import networkx as nx

from topological_action_planner_msgs.msg import Edge, Node
from topological_action_planner_msgs.srv import Plan, PlanRequest, PlanResponse
from topological_action_planner_msgs.srv import UpdateEdge, UpdateEdgeRequest, UpdateEdgeResponse
from visualization_msgs.msg import MarkerArray

from topological_action_planner.ed_interface import EdInterface
from topological_action_planner.serialisation import from_dicts
from topological_action_planner.util import visualize, generate_dummy_graph
from topological_action_planner.visualisation import create_tap_marker_array
from cb_base_navigation_msgs.srv import GetPlan


class TopologicalActionPlanner:
    def __init__(self):
        self.G = from_dicts(rospy.get_param('~edges'))
        self.plot = rospy.get_param('~plot', False)
        self._action_costs = rospy.get_param('~action_costs', {Edge.ACTION_DRIVE: 1,  # per meter distance driven
                                                               Edge.ACTION_OPEN_DOOR: 5,  # per door opened
                                                               Edge.ACTION_PUSH_OBJECT: 10})  # per item pushed

        self.ed = EdInterface('hero', None)

        self._srv_plan = rospy.Service('~get_plan', Plan, self._srv_plan_cb)
        self._srv_update_edge = rospy.Service('~update_edge', UpdateEdge, self._srv_update_edge_cb)
        self._pub_grasp_marker = rospy.Publisher('~graph', MarkerArray, queue_size=10, latch=True)

        # self.G = generate_dummy_graph()
        # to_yaml(self.G, '/tmp/graph.yaml')

        self._global_planner = rospy.ServiceProxy('/global_planner/get_plan_srv', GetPlan)
        if self.plot:
            visualize(self.G)
        self._pub_grasp_marker.publish(create_tap_marker_array(self.G, self.ed))

    def _srv_plan_cb(self, req):
        # type: (PlanRequest) -> PlanResponse

        graph = copy.deepcopy(self.G)  # type: nx.Graph

        for node in graph.nodes.keys():
            graph.nodes[node]['room'] = self.ed.get_room(node[0])  # Based on where they are in ED?

        if req.origin.entity == "":
            # This indicates the plan starts from the robot's current pose
            # We'll insert a node corresponding to our current position.
            # Difficulty is determining it's adjacency to other nodes, based on geometry
            u = 'current', 'current'

            # Ask ED in which room the robot is currently, maybe based on it's pose
            current_room = self.ed.get_room('robot')

            nodes_in_same_room = [node for node, info in graph.nodes.items() if info['room'] == current_room]
            for node in nodes_in_same_room:
                # Will be updated later when taking path length into account later
                graph.add_edge(u, node, action_type=Edge.ACTION_DRIVE, weight=1)
        else:
            u = req.origin.entity, req.origin.area

        v = req.destination.entity, req.destination.area
        rospy.loginfo("Requesting topological action plan from {} to {}".format(u, v))

        try:
            lowest_total_cost = float('inf')
            while True:
                path = nx.shortest_path(graph,
                                        (req.origin.entity, req.origin.area),
                                        (req.destination.entity, req.destination.area),
                                        weight='weight')

                edges = []
                for u, v in zip(path, path[1:]):
                    edges += [Edge(origin=Node(*u),
                                   destination=Node(*v),
                                   cost=graph[u][v]['weight'],
                                   action_type=graph[u][v]['action_type'])]

                # For the drive edges, query the maybe now updated cost of driving that with current knowledge
                for edge in edges:
                    if edge.action_type == Edge.ACTION_DRIVE:
                        src = self.ed.get_center_pose(edge.origin.entity, edge.origin.area)
                        dst = self.ed.get_area_constraint(edge.destination.entity, edge.destination.area)
                        # global_plan = self._global_planner(src, dst)

                        # TODO: get proper cost of action based on total distance instead of amount of poses
                        # edge_cost = len(global_plan.plan) * 0.1 * self._action_costs[Edge.ACTION_DRIVE]
                        edge_cost = 100 * 0.1 * self._action_costs[Edge.ACTION_DRIVE]

                        rospy.loginfo("Updating cost of edge {} - {} = {}"
                                      .format(edge.origin, edge.destination, edge_cost).replace('\n', ', '))
                        graph[(edge.origin.entity, edge.origin.area)]\
                            [(edge.destination.entity, edge.destination.area)]['weight'] = edge_cost
                        edge.cost = edge_cost

                current_total_cost = sum([edge.cost for edge in edges])
                if lowest_total_cost == current_total_cost:
                    rospy.loginfo('The current plan has the lowest cost ({}), picking this'.format(current_total_cost))
                    break  # The cost is not changing anymore, we hit the optimum, use this path
                elif current_total_cost < lowest_total_cost:
                    rospy.loginfo('The current plan has cost {}, lowest is {}. '
                                  'Trying a to find a better plan'.format(current_total_cost, lowest_total_cost))
                    lowest_total_cost = current_total_cost

            rospy.loginfo("Found plan of {} edges".format(len(edges)))
            return PlanResponse(error_msg='', error_code=PlanResponse.SUCCESS, edges=edges)
        except nx.NetworkXNoPath as no_path_found_ex:
            rospy.logerr(no_path_found_ex)
            return PlanResponse(error_msg=str(no_path_found_ex), error_code=PlanResponse.ERROR_NO_PATH_FOUND)
        except nx.NodeNotFound as node_not_found_ex:
            rospy.logerr(node_not_found_ex)
            return PlanResponse(error_msg=str(node_not_found_ex), error_code=PlanResponse.ERROR_UNKNOWN_NODE)

    def _srv_update_edge_cb(self, req: UpdateEdgeRequest) -> UpdateEdgeResponse:
        try:
            self.update_edge(req.updated)
            return UpdateEdgeResponse(success=True)
        except nx.NodeNotFound as node_not_found_ex:
            rospy.logerr(node_not_found_ex)
            return UpdateEdgeResponse(success=False)

    def update_edge(self, edge: Edge):
        u, v = (edge.origin.entity, edge.origin.area), (edge.destination.entity, edge.destination.area)
        self.G[u][v]['weight'] = edge.cost
        self.G[u][v]['action_type'] = edge.action_type


if __name__ == "__main__":
    rospy.init_node("topological_action_planner")

    tap = TopologicalActionPlanner()

    rospy.spin()
