# markerarray = MarkerArray()
# for i in range(10):
#     mark = Marker()
#     mark.header.stamp = rospy.Time.now()
#     mark.header.frame_id = "map"
#     mark.type = Marker.CUBE
#     mark.id = i
#     mark.scale.x = 0.5
#     mark.scale.y = 0.5
#     mark.color.r = 1
#     mark.color.g = 0
#     mark.color.b = 0
#     mark.color.a = 1
#     mark.pose.position.x = i
#     mark.pose.position.y = i
#     markerarray.markers.append(mark)
import rosparam

rosparam.load_file()

def create_tap_markerarray():
    