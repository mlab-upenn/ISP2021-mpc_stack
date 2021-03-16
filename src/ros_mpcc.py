#!/usr/bin/env python3
from track import *
from constraints import *
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry   
from geometry_msgs.msg import Point
from mpcc import *
import rospkg
from params import *
from config import *
from ackermann_msgs.msg import AckermannDriveStamped
rospack = rospkg.RosPack()

class ROS_MPCC:
    def __init__(self):
        self.traj_pub = rospy.Publisher('vis_traj', Marker, queue_size=1)
        self.input_pub = rospy.Publisher('drive',AckermannDriveStamped,queue_size=1)
        # self.mpcc = MPCC(rospack.get_path('mpcc')+ '/data/Silverstone_map_waypoints.csv')
        self.mpcc = MPCC()
        self.param = Params()
        self.x = State()
        self.x.v = self.param.init_vel
        self.x.psi = self.param.init_psi
        self.rate = rospy.Rate(200)
        self.drive_msg = AckermannDriveStamped()
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
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.i = 0

        rospy.Subscriber("odom",Odometry,self.odom_cb)

    def odom_cb(self,msg):
        # pass
        self.x.X = msg.pose.pose.position.x
        self.x.Y = msg.pose.pose.position.y
        self.x.v = max(msg.twist.twist.linear.x,0.1) #To avoid division by zero
        self.x.v_delta = msg.twist.twist.angular.z
        w,z = msg.pose.pose.orientation.w, msg.pose.pose.orientation.z
        t3 = 2.0 * w * z
        t4 = 1.0 - 2.0 * z * z
        self.x.psi = np.arctan2(t3, t4)

    def visualize_traj(self,data):   
        self.marker.points = []
        for i in range(no_of_stages):
            p = Point()
            p.x, p.y = data[i*(n_states_inputs)+3], data[i*(n_states_inputs)+4]
            self.marker.points = self.marker.points + [p]
            self.traj_pub.publish(self.marker)

    def input_publisher(self,data):    
        self.drive_msg.header.frame_id = "laser"
        self.drive_msg.header.stamp = rospy.Time.now()

        # if(self.i < 50):
        #     self.drive_msg.drive.speed = data[6] + 0.5
        #     self.drive_msg.drive.steering_angle = 100 * data[5]
        # else:
        self.drive_msg.drive.speed = data[6]
        self.drive_msg.drive.steering_angle = 10 * data[5]

        self.input_pub.publish(self.drive_msg)
    
    def stop_input_publisher(self):    
        self.drive_msg.header.frame_id = "laser"
        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.drive.speed = 0
        self.drive_msg.drive.steering_angle = 0
        self.input_pub.publish(self.drive_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.i+=1
            data,status = self.mpcc.mpcc_run(self.x)

            if(status != 1):
                self.stop_input_publisher()
                break

            self.visualize_traj(data)
            print(data[5])
            # self.input_publisher(data)
            # if(i==1):
            #     self.input_publisher(data)
            #     np.save('solver_out',data)
            #     print("saved")

            # if(i==2):
            #     print(data)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('mpcc',anonymous = True)
        mpcc = ROS_MPCC()
        mpcc.run()
    except rospy.ROSInterruptException:
        pass