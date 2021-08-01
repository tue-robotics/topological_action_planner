from __future__ import print_function

# TU/e
import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from ed_gui_server_msgs.srv import GetEntityInfo, GetEntityInfoResponse
from ed_msgs.srv import Configure, SimpleQuery, SimpleQueryRequest, UpdateSrv
from ed_navigation_msgs.srv import GetGoalConstraint


def pose_stamped(x, y, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0, frame_id='map'):
    return PoseStamped(header=Header(frame_id=frame_id),
                       pose=Pose(position=Point(x, y, z),
                                 orientation=Quaternion(qx, qy, qz, qw)))


class EdInterface:
    def __init__(self, robot_name, tf_buffer):
        self.robot_name = robot_name
        self.tf_buffer = tf_buffer
        self._ed_simple_query_srv = rospy.ServiceProxy('/%s/ed/simple_query' % robot_name, SimpleQuery)
        self._ed_entity_info_query_srv = rospy.ServiceProxy('/%s/ed/gui/get_entity_info' % robot_name, GetEntityInfo)
        self._get_constraint_srv = rospy.ServiceProxy('/%s/ed/navigation/get_constraint' % robot_name, GetGoalConstraint)

    def get_room(self, entity: str) -> str:
        """In which room is a particular entity?"""

        mapping = {'coffee_table': 'test_area',
                   'biestheuvel_door': 'test_area',
                   'door': 'test_area',
                   'robot': 'test_area'}
        return mapping[entity]

    def get_center_pose(self, entity: str, area: str) -> PoseStamped:
        """What is the center pose of an entity and area?"""

        mapping = {'coffee_table':
                       {'': pose_stamped(2.0, 0.0),
                        'in_front_of': pose_stamped(2.8, 0.0),
                        'on_top_of': pose_stamped(2.0, 0.0)},
                   'door':
                       {'': pose_stamped(1.0, -1.5),
                        'in_front_of': pose_stamped(1.4, -1.5),
                        'in_front_of2': pose_stamped(0.6, -1.5)}}
        return mapping[entity][area]

    def get_area_constraint(self, entity: str, area: str) -> str:
        """What is the center pose of an entity and area?"""
        res = self._get_constraint_srv(entity_ids=[entity],
                                       area_names=[area])
        if not res.error_msg:
            return res.position_constraint_map_frame
        else:
            raise Exception("Cannot get a constraint for {}.{}".format(entity, area))
