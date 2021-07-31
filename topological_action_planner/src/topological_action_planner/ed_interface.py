from __future__ import print_function

# TU/e
import rospy
from ed_gui_server_msgs.srv import GetEntityInfo, GetEntityInfoResponse
from ed_msgs.srv import Configure, SimpleQuery, SimpleQueryRequest, UpdateSrv
# Robot skills
from robot_skills.util.entity import from_entity_info
from robot_skills.util.kdl_conversions import VectorStamped, kdl_vector_to_point_msg


class EdInterface:
    def __init__(self, robot_name, tf_buffer):
        self.robot_name = robot_name
        self.tf_buffer = tf_buffer
        self._ed_simple_query_srv = rospy.ServiceProxy('/%s/ed/simple_query' % robot_name, SimpleQuery)
        self._ed_entity_info_query_srv = rospy.ServiceProxy('/%s/ed/gui/get_entity_info' % robot_name, GetEntityInfo)

    def get_entities(self, type="", center_point=VectorStamped(), radius=float('inf'), id=""):
        """
        Get entities via Simple Query interface

        :param type: Type of entity
        :param center_point: Point from which radius is measured
        :param radius: Distance between center_point and entity
        :param id: ID of entity
        """

        center_point_in_map = center_point.projectToFrame("map", self.tf_buffer)
        query = SimpleQueryRequest(id=id, type=type, center_point=kdl_vector_to_point_msg(center_point_in_map.vector),
                                   radius=radius, ignore_z=True)

        try:
            entity_infos = self._ed_simple_query_srv(query).entities
            entities = list(map(from_entity_info, entity_infos))
        except Exception as e:
            rospy.logerr("ERROR: ed.get_entities(id={}, type={}, center_point={}, radius={})".format(
                id, type, str(center_point), str(radius)))
            rospy.logerr("L____> [%s]" % e)
            return []

        return entities

    def get_closest_entity(self, type="", center_point=None, radius=float('inf')):
        if not center_point:
            center_point = VectorStamped(x=0, y=0, z=0, frame_id=self.robot_name + "/base_link")

        entities = self.get_entities(type=type, center_point=center_point, radius=radius)

        # HACK
        entities = [e for e in entities if e.shape is not None and e.type != ""]

        if len(entities) == 0:
            return None

        # Sort by distance
        try:
            center_in_map = center_point.projectToFrame("map", self.tf_buffer)
            entities = sorted(entities, key=lambda entity: entity.distance_to_2d(center_in_map.vector))
        except Exception as e:
            rospy.logerr("Failed to sort entities: {}".format(e))
            return None

        return entities[0]

    def get_closest_room(self, center_point=None, radius=float('inf')):
        if not center_point:
            center_point = VectorStamped(x=0, y=0, z=0, frame_id="/" + self.robot_name + "/base_link")

        return self.get_closest_entity(type="room", center_point=center_point, radius=radius)
