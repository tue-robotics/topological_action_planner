import unittest
from topological_action_planner.planning import TopoPlanner, EdgeCostCalcBase

import networkx as nx
from typing import Optional

from topological_action_planner_msgs.msg import Edge, Node


class TopoPlannerTest(unittest.TestCase):

    def test_3_node_planning_same_edge_cost(self):
        class SameEdgeCosts(EdgeCostCalcBase):
            def __call__(self, edge: Edge) -> Optional[float]:
                return edge.cost
        topoplanner = TopoPlanner(SameEdgeCosts())

        graph = nx.Graph()
        graph.add_edge(("a", "1"), ("b", "2"), weight=5, action_type=Edge.ACTION_DRIVE)
        graph.add_edge(("b", "2"), ("c", "3"), weight=5, action_type=Edge.ACTION_DRIVE)
        graph.add_edge(
            ("a", "1"),
            ("c", "3"),  # This is the shortest path, directly from a1 to c3
            weight=5,
            action_type=Edge.ACTION_DRIVE,
        )

        a1 = ("a", "1")
        c3 = ("c", "3")
        expected_edges = topoplanner.plan(graph, a1, c3)

        self.assertEqual(len(expected_edges), 1, "Shortest path is 1 edge only")
        self.assertEqual(
            expected_edges,
            [Edge(origin=Node("a", "1"), destination=Node("c", "3"), action_type="DRIVE", cost=5)],
            "Shortest path is from a1 to c3 directly",
        )

    def test_3_node_planning_different_edge_cost(self):
        """
        When planning, the edge weights are recalculated using PredefinedEdgeCosts, increasing the cost of the best plan

        The shortest route in the original graph between a1 to c3 is going directly from a1 to c3.
        But, the edge cost calculator is predefined to 'calculate' a higher cost for this edge, which should change
            the lowest cost path to go via b2.
        """
        class PredefinedEdgeCosts(EdgeCostCalcBase):
            def __init__(self):
                self._map = {(("a", "1"), ("b", "2")): 1,
                             (("b", "2"), ("c", "3")): 1,
                             (("a", "1"), ("c", "3")): 15}

            def __call__(self, edge: Edge) -> Optional[float]:
                return self._map[((edge.origin.entity, edge.origin.area),
                                  (edge.destination.entity, edge.destination.area))]
        topoplanner = TopoPlanner(PredefinedEdgeCosts())

        graph = nx.Graph()
        graph.add_edge(("a", "1"), ("b", "2"), weight=5, action_type=Edge.ACTION_DRIVE)
        graph.add_edge(("b", "2"), ("c", "3"), weight=5, action_type=Edge.ACTION_DRIVE)
        graph.add_edge(
            ("a", "1"),
            ("c", "3"),  # This is the shortest path, directly from a1 to c3
            weight=5,
            action_type=Edge.ACTION_DRIVE,
        )

        a1 = ("a", "1")
        c3 = ("c", "3")
        
        expected_edges = topoplanner.plan(graph, a1, c3)

        self.assertEqual(
            expected_edges,
            [Edge(origin=Node("a", "1"), destination=Node("b", "2"), action_type="DRIVE", cost=1),
             Edge(origin=Node("b", "2"), destination=Node("c", "3"), action_type="DRIVE", cost=1)],
            "Shortest path from a1 to c3 is via b2",
        )
        self.assertEqual(len(expected_edges), 2, "Shortest path should have 2 edges")
