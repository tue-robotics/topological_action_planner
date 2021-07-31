from typing import Sequence, Mapping

import genpy
import networkx as nx
import os
import yaml

from topological_action_planner_msgs.msg import Edge


def as_dicts(graph: nx.Graph):
    edges = []
    for nx_edge in graph.edges.items():
        edge_dict = {'origin': {'entity': nx_edge[0][0][0],
                                'area': nx_edge[0][0][1]},
                     'destination': {'entity': nx_edge[0][1][0],
                                     'area': nx_edge[0][1][1]},
                     'cost': nx_edge[1]['weight'],
                     'action_type': nx_edge[1]['action_type']}
        edges += [edge_dict]


def to_yaml(graph: nx.Graph, filename: str):
    file_dir = os.path.dirname(filename)
    if not os.path.exists(file_dir):
        os.mkdir(file_dir)

    with open(filename, 'w') as storage:  # type: file
        yaml.dump(as_dicts(graph), storage)


def from_dicts(edge_dicts: Sequence[Mapping]):
    graph = nx.Graph()

    for edge_dict in edge_dicts:
        edge = Edge()
        genpy.message.fill_message_args(edge, [edge_dict])  # Convert structure in nested dict to ROS msg

        graph.add_edge((edge.origin.entity, edge.origin.area),
                       (edge.destination.entity, edge.destination.area),
                       weight=edge.cost,
                       action_type=edge.action_type)

    return graph
