#!/usr/bin/env python

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point,Quaternion,Pose
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
#import tf2_ros
import tf as tf2_ros

class OdometryNode:

    def __init__(self):
        # Set publishers
        self.pub_odom = rospy.Publisher('/pf/pose/odom', Odometry, queue_size=1)
        self.listener = tf2_ros.TransformListener()        
        # init internals
        self.last_received_pose = Pose()
        #self.last_received__orientation = Quaternion()

        self.last_recieved_stamp = None

        # Set the update rate
        rospy.Timer(rospy.Duration(.05), self.timer_callback) # 20hz

        # self.tf_pub = tf2_ros.TransformBroadcaster()
        #self.listener = tf2_ros.TransformListener()

        # Set subscribers
        rospy.Subscriber('/odom_rf2o', Odometry, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        trans = Point()
        rot = Quaternion()
        
        # rospy.loginfo("receive rf2o odom")
        print "receive rf2o odom"
        try:
            (trans,rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print "wating for map <==> basefootprint"
            # rospy.loginfo("wating for map <==> basefootprint")
           # pass

        self.last_received_pose.position.x = trans[0]
        self.last_received_pose.position.y = trans[1]
        self.last_received_pose.position.z = trans[2]

        self.last_received_pose.orientation.x = rot[0]
        self.last_received_pose.orientation.y = rot[1]
        self.last_received_pose.orientation.z = rot[2]
        self.last_received_pose.orientation.w = rot[3]
        self.last_recieved_stamp = rospy.Time.now()

    def timer_callback(self, event):
        
        if self.last_recieved_stamp is None:
            return

        print "======on time transmit rf2o odom"

        cmd = Odometry() 
        cmd.header.stamp = self.last_recieved_stamp
        cmd.header.frame_id = "map"#'odom'
        cmd.child_frame_id = 'base_footprint' # This used to be odom

        cmd.pose.pose = self.last_received_pose#.position.x = 0#self.last_received_pose.z
        #cmd.pose.pose.position.y = 0#self.last_received_pose.y
        #cmd.pose.pose.position.z = 0#self.last_received_pose.z
        
        #cmd.pose.pose.orientation = self.last_received__orientation
        # cmd.pose.pose.orientation.y = 1#self.last_received__orientation.y
        # cmd.pose.pose.orientation.z = 1#self.last_received__orientation.z
        # cmd.pose.pose.orientation.w = 1#self.last_received__orientation.w

       
        self.pub_odom.publish(cmd)

        # tf = TransformStamped(
        #     header=Header(
        #         frame_id=cmd.header.frame_id,
        #         stamp=cmd.header.stamp
        #     ),
        #     child_frame_id=cmd.child_frame_id,
        #     transform=Transform(
        #         translation=cmd.pose.pose.position,
        #         rotation=cmd.pose.pose.orientation
        #     )
        # )
        # self.tf_pub.sendTransform(tf)

# Start the node
if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
    rospy.spin()

