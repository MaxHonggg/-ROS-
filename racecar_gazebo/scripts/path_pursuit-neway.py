#!/usr/bin/env python
# sudo cp ./path_pursuit.py /mnt/hgfs/share/
#version 0721:  consider and optimized the situation of no valid goal,we take the "balance" choice,see line 183 -- 204
#version 0721 : wannna more speed!!!!!!
#version 0722: modify the navigation launch file, add ACML node: odom==map???
#version 0722 : no obstacles on the MAP acyually !!!!!!!!!!!!!!!!!!!
import rospy
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import *
from geometry_msgs.msg import PoseStamped, PoseArray,Twist
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64

class following_path:
    def __init__(self):
        #self.current_pose = rospy.Subscriber('/pf/pose/odom', Odometry, self.callback_read_current_position, queue_size=1)
        self.Pose = []
        self.path_pose = rospy.Subscriber('/cmd_vel', Twist, self.callback_read_path, queue_size=1)
        rospy.Timer(rospy.Duration(0.02), self.timer_callback) # 20hz
        
        #global options
        #self.path_global = rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan', Path, self.callback_read_global_path, queue_size=1)
        self.path_global_info = []
        ###################

        #MARKER======================================================================================================
        self.marker_pub = rospy.Publisher('car_path',Marker,queue_size=100)

        self.goal_circle = Marker()
        self.goal_circle.ns = "Markers"
        self.goal_circle.header.frame_id = "map"
        self.goal_circle.action = Marker.ADD
        self.goal_circle.id = 0
        self.goal_circle.type = Marker.SPHERE
        self.goal_circle.pose.orientation.w = 1
        self.goal_circle.scale.x=0.3
        self.goal_circle.scale.y=0.3
        self.goal_circle.scale.z=0.3
        self.goal_circle.color.r = 0.0 
        self.goal_circle.color.g = 0.0 
        self.goal_circle.color.b = 1.0 
        self.goal_circle.color.a = 0.8

        self.goal_circle1 = Marker()
        self.goal_circle1.ns = "Markers"
        self.goal_circle1.header.frame_id = "map"
        self.goal_circle1.action = Marker.ADD
        self.goal_circle1.id = 2
        self.goal_circle1.type = Marker.SPHERE
        self.goal_circle1.pose.orientation.w = 1
        self.goal_circle1.scale.x=0.3
        self.goal_circle1.scale.y=0.3
        self.goal_circle1.scale.z=0.3
        self.goal_circle1.color.r = 0.0 
        self.goal_circle1.color.g = 1.0 
        self.goal_circle1.color.b = 1.0 
        self.goal_circle1.color.a = 0.8

        self.goal_circle2 = Marker()
        self.goal_circle2.ns = "Markers"
        self.goal_circle2.header.frame_id = "map"
        self.goal_circle2.action = Marker.ADD
        self.goal_circle2.id = 3
        self.goal_circle2.type = Marker.SPHERE
        self.goal_circle2.pose.orientation.w = 1
        self.goal_circle2.scale.x=0.3
        self.goal_circle2.scale.y=0.3
        self.goal_circle2.scale.z=0.3
        self.goal_circle2.color.r = 1.0 
        self.goal_circle2.color.g = 1.0 
        self.goal_circle2.color.b = 1.0 
        self.goal_circle2.color.a = 0.8
        #MARKER======================================================================================================
        self.path_info = []
        self.Goal = []
        self.navigation_input = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)
        self.pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)

        self.reach_goal = False
        self.MAX_VELOCITY = 0.5			
        self.MIN_VELOCITY = 0
        self.max_angle = 0.8			#Here!!!!!!!!!!!!!!!!!!!!!!!!!
        self.steering_velocity = 1.5		#Here!!!!!!!!!!!!!!!!!!!!!!!!!
        self.jerk = 0.0
        self.is_local_valid = 1
        self.acceleration = 0.0
        self.LOOKAHEAD_DISTANCE = 0.4
        self.Low_Speed_Mode = False
        self.distance = 0
        self.goal_last_x = 0
        self.goal_last_y = 0
        self.Score = 0
        self.angle = 0
        self.T_front = 0.3
        self.T_rear = 0.3
        self.L = 0.335
        self.pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)
        self.pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)
        self.angle = 1.000   
        self.speed = 0.000
    def timer_callback(self, event):
		
        T_rear = self.T_rear
        T_front = self.T_front
        L = self.L
        r = 0
        print "time on: angle = %f "%(self.angle)

        if math.fabs(self.angle) <1e-5:
            r = 1.0e5
        else:    
            r = L/5*math.fabs(math.tan(self.angle)) 

		# turn radius
        rL_rear = r-(math.copysign(1,self.angle)*(T_rear/2.0))
        rR_rear = r+(math.copysign(1,self.angle)*(T_rear/2.0))
        rL_front = r-(math.copysign(1,self.angle)*(T_front/2.0))
        rR_front = r+(math.copysign(1,self.angle)*(T_front/2.0))

        msgSteerL = Float64()
        msgSteerR = Float64()

        msgSteerL.data = math.atan2(L,rL_front)*math.copysign(1,self.angle)
        self.pub_pos_left_steering_hinge.publish(msgSteerL)

        msgSteerR.data = math.atan2(L,rR_front)*math.copysign(1,self.angle)
        self.pub_pos_right_steering_hinge.publish(msgSteerR)


        msgRearR = Float64()
        msgRearR.data = self.speed * 50 #VELOCITY * rR_rear/r
        self.pub_vel_right_rear_wheel.publish(msgRearR)
        self.pub_vel_right_front_wheel.publish(msgRearR)

        msgRearL = Float64()
        msgRearL.data = self.speed * 50 #VELOCITY * rL_rear/r
        self.pub_vel_left_rear_wheel.publish(msgRearL)
        self.pub_vel_left_front_wheel.publish(msgRearL)

    def callback_read_path(self, data):#local path
		self.angle = data.angular.z
		self.speed = data.linear.x


    def callback_read_global_path(self,data):#global path
        #get the global planner path,x,y,yaw
        self.path_global_info = []
        path_global_array = data.poses
        for path_pose in path_global_array:
            path_x = path_pose.pose.position.x
            path_y = path_pose.pose.position.y
            path_qx = path_pose.pose.orientation.x
            path_qy = path_pose.pose.orientation.y
            path_qz = path_pose.pose.orientation.z
            path_qw = path_pose.pose.orientation.w
            path_quaternion = (path_qx, path_qy, path_qz, path_qw)
            path_euler = euler_from_quaternion(path_quaternion)
            path_yaw = path_euler[2]
            self.path_global_info.append([float(path_x), float(path_y), float(path_yaw)])
        self.Goal = list(self.path_global_info[-1]) # Set the last pose of the global path as goal location	
    def callback_read_current_position(self, data):
        # if self.reach_goal: # Stop updating the information.
        angle = 0
        diff_angle = 0
        path_points_x_list = [float(point[0]) for point in self.path_info]
        path_points_x = np.array(path_points_x_list)

        self.distance = 0
        for id in range(len(path_points_x)-1):
            if id >= len(self.path_global_info):
                break       # avoid out of range
            point = self.path_info[id]
            point_global = self.path_global_info[id]

            # np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
            self.distance += np.sqrt(( point_global[0] - point[0]) ** 2 + ( point_global[1] - point[1]) ** 2) #+( point_global[2] - point[2]) ** 2)

        self.Score = self.distance*len(path_points_x) 

        if  self.Score > 000:		#1500		                		                            #Here!!!!!!!!!!!!!!!!!!!!!!!!!
            self.is_local_valid = 0
        else:
            self.is_local_valid = 1	
        #str="local valid is %d ,goal found : %d" %(self.is_local_valid,goal)
        # str="<<<<<<<<Score is: %d  ,,len_localpath is %d,local valid is %d >>>>>>>>" %(self.distance*len(path_points_x), len(path_points_x),self.is_local_valid)
        # str="<<<<<<<<Score is: %d ,local valid is %d >>>>>>>>" %(self.Score,self.is_local_valid)
        # print(str)	        
        #########################################################            
        while self.reach_goal:
            self.path_info = []
            self.Pose = []
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = 0.0
            ackermann_control.drive.steering_angle = 0.0
            ackermann_control.drive.steering_angle_velocity = 0.0
            self.navigation_input.publish(ackermann_control)
            print('Goal Reached!')
            while True:
                pass
        #########################################################    
        if not len(self.path_info) == 0:
            # Read the path information to path_point list
            if self.is_local_valid == 1:#use local path
                path_points_x_list = [float(point[0]) for point in self.path_info]
                path_points_y_list = [float(point[1]) for point in self.path_info]
                path_points_w_list = [float(point[2]) for point in self.path_info]
                # self.MAX_VELOCITY = 0.4							                            #Here!!!!!!!!!!!!!!!!!!!!!!!!!
                self.LOOKAHEAD_DISTANCE = 0.2						                    #Here!!!!!!!!!!!!!!!!!!!!!!!!!
            else :      #use global path !
                path_points_x_list = [float(point[0]) for point in self.path_global_info]
                path_points_y_list = [float(point[1]) for point in self.path_global_info]
                path_points_w_list = [float(point[2]) for point in self.path_global_info] 
                # self.MAX_VELOCITY = 0.40					                                	#Here!!!!!!!!!!!!!!!!!!!!!!!!!
                self.LOOKAHEAD_DISTANCE = 0.6					                            #Here!!!!!!!!!!!!!!!!!!!!!!!!!

            path_points_x = np.array(path_points_x_list)
            path_points_y = np.array(path_points_y_list)
            path_points_w = np.array(path_points_w_list)

            # Read the current pose of the car from particle filter
            x = data.pose.pose.position.x
            y = data.pose.pose.position.y
            qx = data.pose.pose.orientation.x
            qy = data.pose.pose.orientation.y
            qz = data.pose.pose.orientation.z
            qw = data.pose.pose.orientation.w

            # Convert the quaternion angle to eular angle
            quaternion = (qx,qy,qz,qw)
            euler = euler_from_quaternion(quaternion)
            yaw = euler[2]
            self.Pose = [float(x), float(y), float(yaw)]

            ##############################################
            if self.dist(self.Goal, self.Pose) < 1.0:
                self.Low_Speed_Mode = True
                if self.dist(self.Goal, self.Pose) < 0.3:
                    self.reach_goal = True
                    # print('Goal Reached!')
                else:
                    print('Low Speed Mode ON!')
            else:
                self.Low_Speed_Mode = False
            ###############################################

            # 2. Find the path point closest to the vehichle tat is >= 1 lookahead distance from vehicle's current location.
            dist_array = np.ones(len(path_points_x)+1)
			
            for i in range(len(path_points_x)-2):
                dist_array[i] = self.dist((path_points_x[i], path_points_y[i]), (x,y))

            goal = np.argmin(dist_array)

            # print dist_array
            # print "before querring,goal is dist'smin,index is %d" %(goal)
            # goal = np.where(dist_array == np.argmin(dist_array))[0] # Assume the closet point as the goal point at first
            # goal_array = np.where((dist_array < (self.LOOKAHEAD_DISTANCE + 2.6)) & (dist_array > (self.LOOKAHEAD_DISTANCE)+1))[0]         
           
            id = 0
            # find the start querry position=======
            for id in range(len(self.path_global_info)-1):
                v1 = [path_points_x[id] - x, path_points_y[id] - y]
                v2 = [math.cos(yaw), math.sin(yaw)]
                diff_angle = self.find_angle(v1,v2)
				# judge if the path point if far front of the car
                car2wayPt_x = path_points_x[id] - x 
                car2wayPt_y = path_points_y[id] - y 
                car_theta = yaw
                car_car2wayPt_x = math.cos(car_theta)*car2wayPt_x + math.sin(car_theta)*car2wayPt_y
                car_car2wayPt_y = -1*math.sin(car_theta)*car2wayPt_x + math.cos(car_theta)*car2wayPt_y
                if car_car2wayPt_x >0: 
                    # print "===querry succ at number %d" %(id)
                    break

            if id == len(self.path_global_info)-1:
                print "=== goal start querry fault"
           #=======================================
            start_pos = id

            for id in range(start_pos,len(path_points_x) + start_pos-1):

                if id > len(path_points_x)-1:
                    break

                # print "tring %d times" %(id)

                v1 = [path_points_x[id] - x, path_points_y[id] - y]
                v2 = [math.cos(yaw), math.sin(yaw)]
                diff_angle = self.find_angle(v1,v2)
				# judge if the path point if far front of the car
                car2wayPt_x = path_points_x[id] - x 
                car2wayPt_y = path_points_y[id] - y 
                car_theta = yaw
                car_car2wayPt_x = math.cos(car_theta)*car2wayPt_x + math.sin(car_theta)*car2wayPt_y
                car_car2wayPt_y = -1*math.sin(car_theta)*car2wayPt_x + math.cos(car_theta)*car2wayPt_y
                if car_car2wayPt_x >0 :
					#print "goal is valid!!"
                    if abs(diff_angle) < np.pi/2 and self.dist((path_points_x[id],path_points_y[id]),(x,y)) > self.LOOKAHEAD_DISTANCE: # Check if the one that is the cloest to the lookahead direction
                        goal = id
                        # str="goal found : %d" %(goal)
                        # print(str)                        
                        break
                    # else:
                    #     print "===fault goal,diff_angle too large"
                else:
                    # print "===fault goal,path is behind ,tring %d times" %(id)
                    continue
            
            #  compensation  a little bit 
            # if self.is_local_valid == 0:#use global path
            #     goal += 0
            # print "<<goal is %d>>>" %(goal)
            # if goal == np.argmin(dist_array):           #no valid goal found
            #     # print ">>>>>>>>>>fault<<<<<<<<<with %d"%(len(path_points_x))
            #     print ">>>>>>>>>>>>fault<<<<<<<<<with len(local_path_info):%d,len(self.path_global_info):%d "%(len(self.path_info),len(self.path_global_info))
            #     goal = np.min([len(self.path_info),len(self.path_global_info)])-1   ##TODO:  which one is better choice?????

            # i = goal
            # for i in range(goal,goal + len(self.path_info)):
            #     if dist_array[i] > 0.05 and dist_array[i] < 0.1:
            #         goal2 = i  #position of the far goal = white ball
            #     else:
            #         if dist_array[i] > 0.45 and dist_array[i] < 0.6:   #position of the middle goal =green ball
            #             goal1 = i
            #     print "<= %f =>" %(dist_array[i])

            # print "far ballis  %d, middle is %d" %(goal2,goal1)

        #MARKER======================================================================================================
            self.goal_circle.header.stamp = rospy.Time.now()
            self.goal_circle.pose.position.x=path_points_x[goal]
            self.goal_circle.pose.position.y=path_points_y[goal]
            self.marker_pub.publish(self.goal_circle) 

            self.goal_circle1.pose.position.x=path_points_x[start_pos]#path_points_x[goal]
            self.goal_circle1.pose.position.y=path_points_y[start_pos]#path_points_y[goal]
            self.marker_pub.publish(self.goal_circle1)                        

            # self.goal_circle2.pose.position.x=path_points_x[goal + 60]#path_points_x[goal]
            # self.goal_circle2.pose.position.y=path_points_y[goal + 60]#path_points_y[goal]
            # self.marker_pub.publish(self.goal_circle2)  

            # str="goal is %d,global lenth is %d ,path_points_x is %d,local is %d" %(goal,len(self.path_global_info),len(path_points_x),len(self.path_info))
            # print(str)                                  
        #MARKER======================================================================================================
                            
            VELOCITY = self.speed_control(self.angle, self.MIN_VELOCITY, self.MAX_VELOCITY)

            # 3. Transform the goal point to vehicle coordinates. 
            glob_x = np.array(path_points_x)[goal] - x 
            glob_y = np.array(path_points_y)[goal] - y 
            # goal_x_veh_coord = glob_x*np.cos(yaw) + glob_y*np.sin(yaw)
            # goal_y_veh_coord = glob_y*np.cos(yaw) - glob_x*np.sin(yaw)

            # 4. Calculate the curvature = 1/r = 2x/l^2
            # The curvature is transformed into steering wheel angle by the vehicle on board controller.
            # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
            diff_angle = path_points_w[goal] - yaw # Find the turning angle
            # r = L/(20*np.sin(diff_angle)) # Calculate the turning radius                                       #Here!!!!!!!!!!!!!!!!!!!!!!!!!
            # self.angle = 2 * np.arctan(0.335/r) # Find the wheel turning radius                                      #Here!!!!!!!!!!!!!!!!!!!!!!!!!

            T_rear = self.T_rear
            T_front = self.T_front
            L = self.L

            r = L/math.fabs(math.tan(self.z)) 
			# turn radius
            rL_rear = r-(math.copysign(1,self.z)*(T_rear/2.0))
            rR_rear = r+(math.copysign(1,self.z)*(T_rear/2.0))
            rL_front = r-(math.copysign(1,self.z)*(T_front/2.0))
            rR_front = r+(math.copysign(1,self.z)*(T_front/2.0))

            msgRearR = Float64()
            msgRearR.data = VELOCITY * rR_rear/r
            self.pub_vel_right_rear_wheel.publish(msgRearR)
            self.pub_vel_right_front_wheel.publish(msgRearR)

            msgRearL = Float64()
            msgRearL.data = VELOCITY * rL_rear/r
            self.pub_vel_left_rear_wheel.publish(msgRearL)
            self.pub_vel_left_front_wheel.publish(msgRearL)

            msgSteerL = Float64()
            msgSteerR = Float64()
            
            msgSteerL.data = math.atan2(L,rL_front)*math.copysign(1,self.z)
            self.pub_pos_left_steering_hinge.publish(msgSteerL)

            msgSteerR.data = math.atan2(L,rR_front)*math.copysign(1,self.z)
            self.pub_pos_right_steering_hinge.publish(msgSteerR)

            if id >= len(path_points_x) + start_pos-1:
                self.angle = self.angle_last
                print "========fault goal, use last goal============================================"
            # else:
            #     str="<<<<<Score is %d,local valid is %d ,goal found : %d>>>>>>>>>>" %(self.Score,self.is_local_valid,goal)
            #     print(str)				

        else:
            ackermann_control = AckermannDriveStamped()
            ackermann_control.header.stamp = rospy.Time.now()
            ackermann_control.drive.speed = 0.0
            ackermann_control.drive.steering_angle = 0.0
            ackermann_control.drive.steering_angle_velocity = 0.0
        
        self.angle_last = self.angle        
        # use different topic to get better turn
        if  np.abs(diff_angle)>0.7 and self.Score >660:   #np.abs(diff_angle*self.Score) >2200: #                     #Here!!!!!!!!!!!!!!!!!!!!!!!!!                                                                   #Here!!!!!!!!!!!!!!!!!!!!!!!!!
            ackermann_control.drive.steering_angle = np.sign(self.angle) * 1.06                          #Here!!!!!!!!!!!!!!!!!!!!!!!!!
            self.pub.publish(ackermann_control) 
            print "Extream angle!!!!!!!!!!!!!"
            return
    # Computes the Euclidean distance between two 2D points p1 and p2
    def dist(self, p1, p2):
	try:
		return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)+0.00001
	except:
		return 0.5
    # Compute the angle between car direction and goal direction
    def find_angle(self, v1, v2):
        cos_ang = np.dot(v1, v2)
        sin_ang = LA.norm(np.cross(v1, v2))
        return np.arctan2(sin_ang, cos_ang)
    # Control the speed of the car within the speed limit
    def speed_control(self, angle, MIN_VELOCITY, MAX_VELOCITY):
        # Assume the speed change linearly with respect to yaw angle

        k = (MIN_VELOCITY - MAX_VELOCITY)/self.max_angle + 0.5
        Velocity = self.MAX_VELOCITY		#Here!!!!!!!!!!!!!!!!!!!!!!!!!More Efficient Stratigies!!!!!!!!!!!
        return Velocity 
if __name__ == "__main__":

    rospy.init_node("pursuit_path")
    following_path()
    rospy.spin()


