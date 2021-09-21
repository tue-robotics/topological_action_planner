import copy

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from ed_py.world_model import WM
from geometry_msgs.msg import Point, Pose, Vector3, Quaternion, PoseStamped
import networkx as nx
from pykdl_ros import VectorStamped
import tf2_ros  # noinspection PyUnresolvedReferences
import tf2_geometry_msgs  # noinspection PyUnresolvedReferences
import tf2_pykdl_ros


def create_tap_marker_array(graph: nx.Graph, wm: WM, frame="map") -> MarkerArray:
    """
    Create a MarkerArray representing the graph
    """
    marker_array = MarkerArray()
    stamp = rospy.Time.now()
    rgb_list = {"DRIVE": [0, 0, 1], "OPEN_DOOR": [0, 1, 0], "PUSH_OBJECT": [1, 1, 0]}

    def infinity():
        x = 0
        while True:
            yield x
            x += 1

    inf = infinity()

    for i, (entity_id, area) in enumerate(graph.nodes.keys()):
        entity = wm.get_entity(entity_id)
        if entity is None:
            continue
        n_mark = Marker()
        n_mark.header.stamp = stamp
        n_mark.header.frame_id = frame
        n_mark.type = Marker.SPHERE
        n_mark.ns = "nodes"
        n_mark.id = next(inf)
        n_mark.scale.x = 0.2
        n_mark.scale.y = 0.2
        n_mark.scale.z = 0.2
        n_mark.color.a = 1
        n_mark.color.r = 1

        if area:
            vec = wm.tf_buffer.transform(
                VectorStamped(entity.volumes[area].center_point, frame_id=entity.uuid, stamp=rospy.Time(0)),
                "map",
            )
            n_mark.pose = Pose(
                position=Vector3(vec.vector.x(), vec.vector.y(), vec.vector.z()),
                orientation=Quaternion(0, 0, 0, 1),
            )
        else:
            n_mark.pose = tf2_ros.convert(wm.tf_buffer.transform(entity.pose, "map"), PoseStamped).pose

        marker_array.markers.append(n_mark)

    for j, (origin, destination) in enumerate(graph.edges.keys()):
        e_mark = Marker()
        e_mark.header.stamp = stamp
        e_mark.header.frame_id = frame
        e_mark.type = Marker.ARROW
        e_mark.ns = "edges"
        e_mark.id = next(inf)
        edge_info = graph.edges[origin, destination]
        color_vec = rgb_list[edge_info["action_type"]]
        line_weight = max(min(0.01 * edge_info["weight"], 0.01), 0.1)
        e_mark.scale.x = line_weight
        e_mark.scale.y = line_weight * 2
        e_mark.scale.z = line_weight * 2
        e_mark.color.r = color_vec[0]
        e_mark.color.g = color_vec[1]
        e_mark.color.b = color_vec[2]
        e_mark.color.a = 1
        e_mark.pose.orientation.w = 1  # To squelch annoying messages about uninitialized quats
        origin_entity = wm.get_entity(origin[0])
        if not origin_entity:
            rospy.logwarn("No such entity '{}'".format(origin[0]))
            continue

        if origin[1]:
            vec1 = wm.tf_buffer.transform(
                VectorStamped(
                    origin_entity.volumes[origin[1]].center_point, frame_id=origin_entity.uuid, stamp=rospy.Time(0)
                ),
                "map",
            )
        else:
            vec1 = VectorStamped.from_framestamped(wm.tf_buffer.transform(origin_entity.pose, "map"))

        destination_entity = wm.get_entity(destination[0])
        if not destination_entity:
            rospy.logwarn("No such entity '{}'".format(destination[0]))
            continue

        if destination[1]:
            vec2 = wm.tf_buffer.transform(
                VectorStamped(
                    destination_entity.volumes[destination[1]].center_point,
                    frame_id=destination_entity.uuid,
                    stamp=rospy.Time(0),
                ),
                "map",
            )
        else:
            vec2 = VectorStamped.from_framestamped(wm.tf_buffer.transform(destination_entity.pose, "map"))
        e_mark.points = [
            Point(vec1.vector.x(), vec1.vector.y(), vec1.vector.z()),
            Point(vec2.vector.x(), vec2.vector.y(), vec2.vector.z()),
        ]
        marker_array.markers.append(e_mark)

        reversed_arrow = copy.deepcopy(e_mark)
        reversed_arrow.id = next(inf)
        reversed_arrow.points = list(reversed(reversed_arrow.points))
        marker_array.markers.append(reversed_arrow)

        rospy.loginfo("Add edge {} -> {}".format(origin, destination))
        rospy.loginfo("{} -> {}".format(origin_entity, destination_entity))
        rospy.loginfo("{} -> {}".format(vec1, vec2))

    return marker_array
