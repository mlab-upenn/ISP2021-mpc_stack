#!/usr/bin/env python3
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospkg 

rospack = rospkg.RosPack()

def centerline_publisher():
    pub = rospy.Publisher('centerline', Marker, queue_size=1)
    rospy.init_node('centerline_publisher',anonymous = True)
    rate = rospy.Rate(10)
    
    data = np.genfromtxt(rospack.get_path('mpcc')+ '/data/IMS_centerline.csv', delimiter=',')
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.color.a = 1.0 
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0

    
    for i in range(data.shape[0]):
        p = Point()
        p.x = data[i,0]
        p.y = data[i,1]
        p.z = 0
        marker.points = marker.points + [p]


    while not rospy.is_shutdown():
        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        centerline_publisher()
    except rospy.ROSInterruptException:
        pass
