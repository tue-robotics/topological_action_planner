import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point


def create_tap_marker_array(graph, ed, frame="map"):
    def get_entity_pos(entity, area):
        ent = ed.get_entity(entity)
        return ent.volumes[area].center_point

    marker_array = MarkerArray()
    stamp = rospy.Time.now()

    for i, (entity, area) in enumerate(graph.nodes.keys()):
        n_mark = Marker()
        n_mark.header.stamp = stamp
        n_mark.header.frame_id = frame
        n_mark.type = Marker.CUBE
        n_mark.id = i
        n_mark.scale.x = 0.1
        n_mark.scale.y = 0.1
        n_mark.color.r = 1
        pos = get_entity_pos(entity, area)
        n_mark.pose.position.x = pos.x
        n_mark.pose.position.y = pos.y
        marker_array.markers.append(n_mark)

    for j, (origin, destination) in enumerate(graph.edges.keys()):
        e_mark = Marker()
        e_mark.header.stamp = stamp
        e_mark.header.frame_id = frame
        e_mark.type = Marker.LINE_LIST
        e_mark.id = j
        e_mark.scale.x = 0.1
        e_mark.scale.y = 0.1
        e_mark.color.b = 1
        e_mark.color.a = 1
        e_mark.points = []
        first_point = Point()
        pos1 = get_entity_pos(origin.entity, origin.area)
        first_point.x = pos1.x
        first_point.y = pos1.y
        second_point = Point()
        pos2 = get_entity_pos(destination.entity, destination.area)
        second_point.x = pos2.x
        second_point.y = pos2.y
        e_mark.points.append(first_point)
        e_mark.points.append(second_point)
        marker_array.markers.append(e_mark)

    return marker_array
