# !/usr/bin/env python
# license removed for brevity

#TODO after the visualisation is tested this file can be removed

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point


def pub_marker():
    pub = rospy.Publisher('TAP_Marker', MarkerArray, queue_size=10)
    rospy.init_node('tap_marker_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        markerarray = MarkerArray()
        for i in range(10):
            mark = Marker()
            mark.header.stamp = rospy.Time.now()
            mark.header.frame_id = "map"
            mark.type = Marker.CUBE
            mark.id = i
            mark.scale.x = 0.5
            mark.scale.y = 0.5
            mark.color.r = 1
            mark.color.a = 1
            mark.pose.position.x = i
            mark.pose.position.y = i
            markerarray.markers.append(mark)
        rospy.loginfo(markerarray)
        pub.publish(markerarray)
        rate.sleep()


if __name__ == '__main__':
    try:
        pub_marker()
    except rospy.ROSInterruptException:
        pass
