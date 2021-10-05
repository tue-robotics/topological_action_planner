import unittest
import mock
from topological_action_planner.planning import TopoPlanner, EdgeCostCalcBase

import networkx as nx
from typing import Optional

from topological_action_planner_msgs.msg import Edge, Node


class TopoPlannerTest(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        class SameEdgeCosts(EdgeCostCalcBase):
            # def __init__(self):
            #     self._costs = {(('a', '1'), ('b', '2')): 10,
            #                    (('b', '2'), ('c', '3')): 20,
            #                    }
            def __call__(self, edge: Edge) -> Optional[float]:
                return edge.cost

        self.same_cost_calc = SameEdgeCosts()

    def test_3_node_planning(self):
        topoplanner = TopoPlanner(self.same_cost_calc)

        graph = nx.Graph()
        graph.add_edge(("a", "1"), ("b", "2"), weight=5, action_type=Edge.ACTION_PUSH_OBJECT)
        graph.add_edge(("b", "2"), ("c", "3"), weight=5, action_type=Edge.ACTION_PUSH_OBJECT)
        graph.add_edge(
            ("a", "1"),
            ("c", "3"),  # This is the shortest path, directly from a1 to c3
            weight=5,
            action_type=Edge.ACTION_PUSH_OBJECT,
        )

        a1 = ("a", "1")
        b2 = ("b", "2")
        c3 = ("c", "3")
        expected_edges = topoplanner.plan(graph, a1, c3)

        self.assertEqual(len(expected_edges), 1, "Shortest path is 1 edge only")
        self.assertEqual(
            expected_edges,
            [Edge(origin=Node("a", "1"), destination=Node("c", "3"), action_type="PUSH_OBJECT", cost=5)],
            "Shortest path is from a1 to c3",
        )
