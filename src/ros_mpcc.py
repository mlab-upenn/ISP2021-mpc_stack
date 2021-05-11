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
import time
rospack = rospkg.RosPack()

class ROS_MPCC:
    def __init__(self):
        # Trajectory and control publishers
        self.traj_pub = rospy.Publisher('vis_traj', Marker, queue_size=1)
        self.input_pub = rospy.Publisher('drive',AckermannDriveStamped,queue_size=1)

        # Change the map file for different maps
        self.track = Track(rospack.get_path('mpcc')+'/data/IMS_centerline.csv')
        self.mpcc = MPCC(rospack.get_path('mpcc')+'/data/IMS_centerline.csv')
        self.param = Params()
        
        self.x = State()                                    # State variale for the optimization
        self.x.v = self.param.init_vel
        self.i = 0                                          # Iteration counter
        self.last_time = 0                                  # Time variable for controller frequency
        self.init_odom = 0                                  # Flag to check if the odom is initialized

        self.prev_update_time = 0

        self.rate = rospy.Rate(200)
        self.drive_msg = AckermannDriveStamped()

        # Odometry subscriber
        rospy.Subscriber("odom",Odometry,self.odom_cb)

        # Marker messages for the trajectory visualization
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

# Odom callback function
    def odom_cb(self,msg):

        self.x.X = msg.pose.pose.position.x
        self.x.Y = msg.pose.pose.position.y
        temp_s = self.track.get_proj(self.x.X,self.x.Y)

        self.x.v = np.clip(msg.twist.twist.linear.x, 0, self.param.v_ub)
        self.x.v_delta = np.clip(msg.twist.twist.angular.z, self.param.v_delta_lb,self.param.v_delta_ub)

        w,z = msg.pose.pose.orientation.w, msg.pose.pose.orientation.z
        t3 = 2.0 * w * z
        t4 = 1.0 - 2.0 * z * z
        temp_psi = np.arctan2(t3, t4)
        
        if (self.init_odom == 0):
            self.init_odom = 1
            self.prev_update_time = time.time()
        else:

            delta_psi = temp_psi - self.x.psi

            # Wrap the psi_dot
            if(delta_psi > np.pi):
                delta_psi = delta_psi - 2 * np.pi
            elif(delta_psi < - np.pi):
                delta_psi = delta_psi + 2 * np.pi
            
            temp_time = time.time()
            delta_time = temp_time - self.prev_update_time
            self.prev_update_time = temp_time

            # self.x.psi_dot = np.clip(delta_psi / delta_time,self.param.psi_dot_lb,self.param.psi_dot_ub)
            # self.x.v_s = np.clip((temp_s - self.x.s)/delta_time,self.param.v_s_lb,self.param.v_s_ub)
            # self.x.delta  = np.clip(self.x.delta + self.x.v_delta * delta_time, self.param.delta_lb, self.param.delta_ub)

        self.x.psi = temp_psi
        self.x.s = temp_s
        
    # Publish trajectory visulaization messages
    def visualize_traj(self,data):   
        self.marker.points = []
        for i in range(no_of_stages):
            p = Point()
            p.x, p.y = data[i*(n_states_inputs)+3], data[i*(n_states_inputs)+4]
            self.marker.points = self.marker.points + [p]
            self.traj_pub.publish(self.marker)

    # Publish control command
    def input_publisher(self,data):
        if(self.i == 1):
            self.last_time = time.time()

        self.drive_msg.header.frame_id = "laser"
        self.drive_msg.header.stamp = rospy.Time.now()

        self.drive_msg.drive.speed = data[13+7-1]
        self.drive_msg.drive.steering_angle = data[13+6-1]

        curr_time = time.time()
        
        print("control time",curr_time - self.last_time)
        print("iter",self.i)

        print("Steer from solver ",self.drive_msg.drive.steering_angle)
        print("Steer from simulator", self.x.delta)

        print("Velocity from solver ",self.drive_msg.drive.speed)
        print("Velocity from simulator", self.x.v)

        print("Steer velocity from solver ",data[12])
        print("Steer velocity from simulator", self.x.v_delta)
        
        print("Psi from solver ",data[7])
        print("Psi from simulator",self.x.psi)

        print("Psi velocity from solver ",data[8])
        print("Psi velocity from simulator",self.x.psi_dot)
        
        # print("s from solver ",data[10])
        # print("s from simulator", self.x.s)

        self.x.v_s = data[11]
        # print("Virtual velocity from solver",data[11])
        # print("Virtual velocity from simulator", self.x.v_s)

        self.last_time = curr_time
        self.input_pub.publish(self.drive_msg)
    
    def stop_input_publisher(self):    
        self.drive_msg.header.frame_id = "laser"
        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.drive.speed = 0
        self.drive_msg.drive.steering_angle = 0
        self.input_pub.publish(self.drive_msg)

    # Main function to run the solver
    def run(self):
        while not rospy.is_shutdown():
            if(self.init_odom == 0):
                continue
            self.i+=1
            data,status = self.mpcc.mpcc_run(self.x)

            if(status != 1):
                self.stop_input_publisher()
                print("Status ",status)
                rospy.signal_shutdown("Solver failed")

            self.visualize_traj(data)
            
            self.input_publisher(data)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('mpcc',anonymous = True)
        mpcc = ROS_MPCC()
        mpcc.run()
    except rospy.ROSInterruptException:
        pass