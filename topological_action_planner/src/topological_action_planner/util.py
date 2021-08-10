import networkx as nx

from topological_action_planner_msgs.msg import Edge


def generate_dummy_graph(n_nodes=10, n_edges=20):
    numbers = "0123456789"
    letters = "abcdefghijklmnopqrstuvwxyz"
    import random

    random.seed(123456789)
    # Make a bunch of nodes with randomly picked names from numbers and letters
    nodes = [(random.choice(numbers), random.choice(letters)) for _ in range(n_nodes)]

    graph = nx.Graph()

    for _ in range(n_edges):
        a, b = random.choice(nodes), random.choice(nodes)
        if a == b:
            continue
        graph.add_edge(
            a,
            b,
            weight=random.random(),
            # 60% chance of driving, 30% doors, 10% pushing:
            action_type=random.choice(
                [Edge.ACTION_DRIVE] * 6 + [Edge.ACTION_OPEN_DOOR] * 3 + [Edge.ACTION_PUSH_OBJECT]
            ),
        )

    return graph


def visualize(graph):
    import matplotlib.pyplot as plt

    nx.draw(graph, with_labels=True)
    plt.ion()
    plt.show()
    plt.pause(0.001)
