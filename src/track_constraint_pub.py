#!/usr/bin/env python3
from track import *
from constraints import *
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import rospkg

rospack = rospkg.RosPack()

class Track_con_vis:
    def __init__(self):
        self.track = Track(rospack.get_path('mpcc')+ '/data/IMS_centerline.csv')
        self.constraints = Constraints()
        self.rate = rospy.Rate(10)

        self.pub = rospy.Publisher('track_constraints', Marker, queue_size=1)

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.id = 0
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 0.3
        self.marker.scale.y = 0.3
        self.marker.color.a = 1.0 
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        rospy.Subscriber("odom",Odometry,self.track_con_pub)
    
    def track_con_pub(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        dx,dy,o_x,o_y,i_x,i_y = self.constraints.vis_track_constraints(x,y,self.track)

        factor = np.linspace(-2,2,20)

        outer_line_x, outer_line_y = o_x + factor * dx , o_y + factor * dy
        inner_line_x, inner_line_y = i_x + factor * dx , i_y + factor * dy

        for i in range(factor.shape[0]):
            outer_p, inner_p = Point(), Point()
            outer_p.x, outer_p.y = outer_line_x[i],outer_line_y[i]
            inner_p.x, inner_p.y = inner_line_x[i],inner_line_y[i]
            self.marker.points = self.marker.points + [outer_p,inner_p]  
        self.pub.publish(self.marker)
        self.marker.points = []

if __name__ == '__main__':
    try:
        rospy.init_node('track_constraints_publisher',anonymous = True)
        con = Track_con_vis()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass