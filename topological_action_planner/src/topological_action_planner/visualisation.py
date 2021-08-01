import rospy
from visualization_msgs.msg import MarkerArray, Marker
from ed_py.world_model import WM
from geometry_msgs.msg import Point


def create_tap_marker_array(graph, wm: WM, frame="map"):
    marker_array = MarkerArray()
    stamp = rospy.Time.now()

    for i, (entity, area) in enumerate(graph.nodes.keys()):
        n_mark = Marker()
        n_mark.header.stamp = stamp
        n_mark.header.frame_id = frame
        n_mark.type = Marker.SPHERE
        n_mark.ns = 'nodes'
        n_mark.id = i
        n_mark.scale.x = 0.1
        n_mark.scale.y = 0.1
        n_mark.scale.z = 0.1
        n_mark.color.a = 1
        n_mark.color.r = 1
        n_mark.pose = wm.get_entity(entity).volumes[area].center_point
        marker_array.markers.append(n_mark)

    for j, (origin, destination) in enumerate(graph.edges.keys()):
        e_mark = Marker()
        e_mark.header.stamp = stamp
        e_mark.header.frame_id = frame
        e_mark.type = Marker.LINE_LIST
        e_mark.ns = 'edges'
        e_mark.id = j + len(graph.nodes.keys())
        e_mark.scale.x = 0.1
        e_mark.scale.y = 0.1
        e_mark.scale.z = 0.1
        e_mark.color.b = 1
        e_mark.color.a = 1
        e_mark.pose.orientation.w = 1  # To squelch annoying messages about uninitialized quats

        vec1 = wm.get_entity(origin[0]).volumes[origin[1]].center_point,
        vec2 = wm.get_entity(destination[0]).volumes[destination[1]].center_point
        e_mark.points = [Point(vec1.x, vec1.y, vec1.z),
                         Point(vec2.x, vec2.y, vec2.z)]
        marker_array.markers.append(e_mark)

    return marker_array
