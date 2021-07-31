#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def pub_marker():
    pub = rospy.Publisher('TAP_Marker', Marker, queue_size=10)
    rospy.init_node('TAP Marker publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        mark = Marker()
        mark.header.stamp = rospy.Time.now()
        mark.header.frame_id = "map"
        mark.type = 4
        mark.scale.x = 0.5
        mark.scale.y = 0.5
        mark.color.r = 1
        mark.color.g = 0
        mark.color.b = 0
        mark.color.a = 1
        points_to_add = []
        for i in range(10):
            p = Point()
            p.x = i
            p.y = i
            points_to_add.append(p)
        mark.points = points_to_add
        rospy.loginfo(mark)
        pub.publish(mark)
        rate.sleep()


if __name__ == '__main__':
    try:
        pub_marker()
    except rospy.ROSInterruptException:
        pass
