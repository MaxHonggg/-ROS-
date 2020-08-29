/*
Copyright (c) 2017, ChanYuan KUO, YoRu LU,
latest editor: HaoChih, LIN
All rights reserved. (Hypha ROS Workshop)

This file is part of hypha_racecar package.

hypha_racecar is free software: you can redistribute it and/or modify
it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published
by the Free Software Foundation, either version 3 of the License, or
any later version.

hypha_racecar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.

You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with hypha_racecar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int8.h>

#define PI 3.14159265358979
#define rad2deg 57.3248

/********************/
/* CLASS DEFINITION */
/********************/
int sign(const double x)
{
    if(x<0)
        return -1;
    else
        if(x>0)
            return 1;
    return 0;
}
class StanleyController
{
	public:
		StanleyController();
		double calc_target_index(bool& is_positive);
        void initMarker();
		double getdistance_car2point(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        double getYawFromPose(const geometry_msgs::Pose& carPose);
        double getCar2GoalDist();

    private:
        ros::NodeHandle n_;
        ros::Subscriber odom_sub, path_sub, goal_sub,goal_region_sub;
        ros::Publisher PID_pub,goal_notify_pub,pub_, marker_pub,pub_vel_left_rear_wheel,pub_vel_right_rear_wheel,pub_vel_left_front_wheel,pub_vel_right_front_wheel,pub_pos_left_steering_hinge,pub_pos_right_steering_hinge;
        ros::Timer timer1, timer2;
        tf::TransformListener tf_listener;
        visualization_msgs::Marker points, line_strip, goal_circle;
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Point odom_goal_pos;
        nav_msgs::Odometry odom;
        nav_msgs::Path map_path;
        std_msgs::Int8 goal_region;
        int controller_freq, baseSpeed;
        bool foundForwardPt, goal_received, goal_reached;
        double goalRadius;
		double k,v;

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void goalReachingCB(const ros::TimerEvent&);
        void controlLoopCB(const ros::TimerEvent&);
        void goal_region_receive(const std_msgs::Int8& receive_goal_region);

};
double StanleyController::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);

    return dist2goal;
}
StanleyController::StanleyController()
{
    //Private parameters handler
    ros::NodeHandle pn;
	/***** 控制参数  *******/
    /*if (pn.getParam("stanley_k", k))
    {
        pn.param("stanley_v", v, 0.005);
        ROS_INFO_STREAM("Stanley parameters Loaded ");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load Stanley parameters");
        n_.shutdown();
    }	*/
	/**********************/

    odom_sub = n_.subscribe("/pf/pose/odom", 1, &StanleyController::odomCB, this);
    path_sub = n_.subscribe("/move_base/TebLocalPlannerROS/global_plan", 1, &StanleyController::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &StanleyController::goalCB, this);
    goal_region_sub = n_.subscribe("/goal_region",1,&StanleyController::goal_region_receive,this);

    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);
    goal_notify_pub = n_.advertise<std_msgs::Int8>("/L1_controller_v2/goal_finish",1);

	pub_vel_left_rear_wheel = n_.advertise<std_msgs::Float64>("/racecar/left_rear_wheel_velocity_controller/command", 1);
    pub_vel_right_rear_wheel = n_.advertise<std_msgs::Float64>("/racecar/right_rear_wheel_velocity_controller/command", 1);
    pub_vel_left_front_wheel = n_.advertise<std_msgs::Float64>("/racecar/left_front_wheel_velocity_controller/command", 1);
    pub_vel_right_front_wheel = n_.advertise<std_msgs::Float64>("/racecar/right_front_wheel_velocity_controller/command", 1);
    pub_pos_left_steering_hinge = n_.advertise<std_msgs::Float64>("/racecar/left_steering_hinge_position_controller/command", 1);
    pub_pos_right_steering_hinge = n_.advertise<std_msgs::Float64>("/racecar/right_steering_hinge_position_controller/command", 1);

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &StanleyController::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5)/controller_freq), &StanleyController::goalReachingCB, this); // Duration(0.05) -> 20Hz

    //Init variables
    initMarker();
    //Show info
    ROS_INFO("[param] stanley_k: %f", k);
    ROS_INFO("[param] stanley_v: %f", v);
    goalRadius = 1;

}
double StanleyController::getdistance_car2point(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    double dx = wayPt.x - carPose.position.x;
    double dy = wayPt.y - carPose.position.y;

    return (double)(sqrt(dx*dx + dy*dy));

}

double StanleyController::calc_target_index(bool& is_positive)
{	//搜索最临近的路点，并指出临近点是在左侧+还是右侧-

	double distance_min = 1.5e3 ,distance_now = 0.0;
    int index = 0;

	is_positive = false;
	geometry_msgs::Pose carPose = odom.pose.pose;
	for(int i =0; i< map_path.poses.size(); i++)
	{
		geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
		geometry_msgs::PoseStamped odom_path_pose;

		try
		{
			tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
			geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
			distance_now = getdistance_car2point(odom_path_wayPt,carPose);
			if(distance_now < distance_min)
			{
				distance_min = distance_now;
				index = i;
			}
		}
		catch(tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	points.points.push_back(map_path.poses[index].pose.position);//????????是否是这个？
    marker_pub.publish(points);

	if(map_path.poses[index].pose.position.x >  carPose.position.x)
		is_positive = true;

    return distance_min;

}
void StanleyController::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;


    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}
void StanleyController::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    // ROS_INFO("odomCB");

    odom = *odomMsg;
}
void StanleyController::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    // ROS_INFO("pathCB");
    map_path = *pathMsg;
}
void StanleyController::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{

    // ROS_INFO("goalCB");

    // while(1)
    // {
        tf_listener.waitForTransform("/map", "/odom", ros::Time::now(), ros::Duration(4.0));
        try
        {
            ros::Time now = ros::Time::now();
            tf_listener.waitForTransform("/map", "/odom" , now , ros::Duration(4.0));

            geometry_msgs::PoseStamped odom_goal;
            tf_listener.transformPose("odom", *goalMsg ,odom_goal);
            odom_goal_pos = odom_goal.pose.position;
            goal_received = true;
            goal_reached = false;
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.5).sleep();
        }
    // }
}
void StanleyController::goal_region_receive(const std_msgs::Int8& receive_goal_region)
{
    goal_region = receive_goal_region;
    ROS_INFO("Goal Region recived : %d!",goal_region.data);

}
void StanleyController::goalReachingCB(const ros::TimerEvent&)
{

    if(goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        if(car2goal_dist < goalRadius)
        {
            goal_reached = true;
            goal_received = false;
            ROS_INFO("Goal Reached !");

            goal_notify_pub.publish(goal_region);
        }
    }
}
double StanleyController::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}

void StanleyController::controlLoopCB(const ros::TimerEvent&)
{
	bool is_point_positive;
	double error = calc_target_index(is_point_positive);
	double cross_track_angle = 0.0;
    geometry_msgs::Pose carPose = odom.pose.pose;

    double carPose_yaw = getYawFromPose(carPose);

	if(is_point_positive == false)
        error *= -1;

/*
    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
        th = ch[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        th = ch[-1]
        ind = len(cx) - 1*/

	//计算目标点切线夹角cross_track_angle


    // 计算横向误差
    double delta = cross_track_angle - carPose_yaw + atan2(k * error, v);


}

//=======================================================
class L1Controller
{
    public:
        L1Controller();
        void initMarker();
        int isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose,float&);
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
        bool isWayPtAwayFromLrvDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& car_pos);
        double getYawFromPose(const geometry_msgs::Pose& carPose);
        double getEta(const geometry_msgs::Pose& carPose);
        double getEta2(const geometry_msgs::Pose& carPose);
        double getCar2GoalDist();
        double getL1Distance(const double& _Vcmd);
        double getSteeringAngle(double eta);
        double getSteeringAngle2(double eta2);
        double getGasInput(const float& current_v);
        geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);
        geometry_msgs::Point get_odom_car2WayPtVec2(const geometry_msgs::Pose& carPose);

    private:
        ros::NodeHandle n_;
        ros::Subscriber odom_sub, path_sub, goal_sub,goal_region_sub, move_base_cmdvel_sub;
        ros::Publisher PID_pub,goal_notify_pub,pub_, marker_pub,pub_vel_left_rear_wheel,pub_vel_right_rear_wheel,pub_vel_left_front_wheel,pub_vel_right_front_wheel,pub_pos_left_steering_hinge,pub_pos_right_steering_hinge;
        ros::Timer timer1, timer2;
        tf::TransformListener tf_listener;

        visualization_msgs::Marker points, line_strip, goal_circle,points_foward;
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Point odom_goal_pos;
        nav_msgs::Odometry odom;
        nav_msgs::Path map_path;
        std_msgs::Int8 goal_region;
        double Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v,kp,ki,kd;
        double Gas_gain, baseAngle, Angle_gain, goalRadius;
        int controller_freq, baseSpeed;
        bool foundForwardPt, goal_received, goal_reached;
        double steer_eta_last,steer_eta_last2,steer_eta_integral,steer_eta_integral2;
        bool Reverse_Drive_Flag,Reverse_Drive_Flag2;
        double T_front;
        double T_rear;
        double L;
        double Reverse_angle;
        double Reverse_speed;
        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void goalReachingCB(const ros::TimerEvent&);
        void controlLoopCB(const ros::TimerEvent&);
        void goal_region_receive(const std_msgs::Int8& receive_goal_region);
        void cmd_vel_receive(const geometry_msgs::Twist& );
}; // end of class


L1Controller::L1Controller()
{
    //Private parameters handler
    ros::NodeHandle pn("L1_controller_v2");

    //Car parameter
    pn.param("L", L, 0.335);
    pn.param("Lrv", Lrv, 1.0);
    pn.param("Vcmd", Vcmd, 1.0);
    pn.param("lfw", lfw, 0.1675);
    pn.param("lrv", lrv, 0.1675);//10.0


    pn.param("kp", kp, 3.0);
    pn.param("ki", ki, 0.005);
    pn.param("kd", kd, 1.0);



    {
    //Controller parameter
    pn.param("controller_freq", controller_freq, 30);
    pn.param("AngleGain", Angle_gain, -1.0);
    pn.param("GasGain", Gas_gain, 1.0);
    pn.param("baseSpeed", baseSpeed, 1470);
    pn.param("baseAngle", baseAngle, 90.0);

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/pf/pose/odom", 1, &L1Controller::odomCB, this);
    path_sub = n_.subscribe("/move_base/TebLocalPlannerROS/global_plan", 1, &L1Controller::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
    goal_region_sub = n_.subscribe("/goal_region",1,&L1Controller::goal_region_receive,this);
    move_base_cmdvel_sub = n_.subscribe("/cmd_vel",100,&L1Controller::cmd_vel_receive,this);

    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);
    goal_notify_pub = n_.advertise<std_msgs::Int8>("/L1_controller_v2/goal_finish",1);
    PID_pub = n_.advertise<std_msgs::Float64>("err",1);

	pub_vel_left_rear_wheel = n_.advertise<std_msgs::Float64>("/racecar/left_rear_wheel_velocity_controller/command", 1);
    pub_vel_right_rear_wheel = n_.advertise<std_msgs::Float64>("/racecar/right_rear_wheel_velocity_controller/command", 1);
    pub_vel_left_front_wheel = n_.advertise<std_msgs::Float64>("/racecar/left_front_wheel_velocity_controller/command", 1);
    pub_vel_right_front_wheel = n_.advertise<std_msgs::Float64>("/racecar/right_front_wheel_velocity_controller/command", 1);
    pub_pos_left_steering_hinge = n_.advertise<std_msgs::Float64>("/racecar/left_steering_hinge_position_controller/command", 1);
    pub_pos_right_steering_hinge = n_.advertise<std_msgs::Float64>("/racecar/right_steering_hinge_position_controller/command", 1);

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &L1Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5)/controller_freq), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz

    //Init variables
    goalRadius = 1.5;
    Lfw = getL1Distance(Vcmd);
    foundForwardPt = false;
    Reverse_Drive_Flag = false;
    Reverse_Drive_Flag2 = false;
    goal_received = false;
    goal_reached = false;
    cmd_vel.linear.x = 1500; // 1500 for stop
    cmd_vel.angular.z = baseAngle;
    steer_eta_last = 0;
    steer_eta_integral = 0;

    T_front = 0.3;
    T_rear = 0.3;
    L = 0.335;
    Reverse_angle = 0;
    Reverse_speed = 0;

    //Show info

    ROS_INFO("[param] Lrv: %f", Lrv);
    ROS_INFO("[param] Lfw: %f", Lfw);
    
    ROS_INFO("[param] lrv: %f", lrv);
    ROS_INFO("[param] lfw: %f", lfw);

    ROS_WARN("------------------------");
    
    ROS_WARN("[param] kp: %f", kp);
    ROS_WARN("[param] ki: %f", ki);
    ROS_WARN("[param] kd: %f", kd);
    ROS_WARN("[param] controller_freq: %d", controller_freq);
    
    ROS_WARN("------------------------");
    //Visualization Marker Settings
    initMarker();
    }
}
void L1Controller::initMarker()
{
    points_foward.header.frame_id = points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points_foward.ns = points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points_foward.action = points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points_foward.pose.orientation.w = points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;
    points_foward.id = 3;

    points.type = visualization_msgs::Marker::POINTS;
    points_foward.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points_foward.scale.x = points.scale.x = 0.2;
    points_foward.scale.y = points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goalRadius;
    goal_circle.scale.y = goalRadius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    points_foward.color.r = 1.0;
    points_foward.color.g = 1.0;
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}
void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    // ROS_INFO("odomCB");
    odom = *odomMsg;  //车位子
}
void L1Controller::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    // ROS_INFO("pathCB");
    map_path = *pathMsg; //全局路径
}
void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{

    // ROS_INFO("goalCB");

    // while(1)
    // {
        tf_listener.waitForTransform("/map", "/odom", ros::Time::now(), ros::Duration(4.0));
        try
        {
            ros::Time now = ros::Time::now();
            tf_listener.waitForTransform("/map", "/odom" , now , ros::Duration(4.0));

            geometry_msgs::PoseStamped odom_goal;
            tf_listener.transformPose("odom", *goalMsg ,odom_goal);
            odom_goal_pos = odom_goal.pose.position;
            goal_received = true;
            goal_reached = false;

            /*Draw Goal on RVIZ*/
            // goal_circle.pose = odom_goal.pose;
            // marker_pub.publish(goal_circle);
            // break;
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.5).sleep();
        }
    // }
}
double L1Controller::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}
int L1Controller::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose,float& delta)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);
	double goal_theta = atan2(wayPt.y-carPose.position.y,wayPt.x-carPose.position.x);
	

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;//odom to car
	
	delta = goal_theta - car_theta;
	if(fabs(delta) > PI)
		delta = PI*2 - delta*sign(delta);
	

 	//ROS_INFO("goal_theta = %f,car_theta=%f,delta=%f",goal_theta,car_theta,delta);

    // if(car_car2wayPt_x  > 0 && fabs(delta*rad2deg) < 45) /*is Forward WayPt and in the turn angle?*/
        return 3;//正常前进
    // else
    //     if(car_car2wayPt_x  < 0)
    //         return 2;//使用倒车控制算法

    //     return 1;//cmd_vel
}
 
bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < Lfw)
        return false;
    else if(dist >= Lfw && dist < Lfw*2 )
        return true;
}
bool L1Controller::isWayPtAwayFromLrvDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& car_pos)
{
    double car_theta = getYawFromPose(car_pos);
    geometry_msgs::Point carPose_pos = car_pos.position;

    float car_car_pos_x = cos(car_theta)*carPose_pos.x + sin(car_theta)*carPose_pos.y;
    float car_car_pos_y = -sin(car_theta)*carPose_pos.x + cos(car_theta)*carPose_pos.y;
    float new_car_car_pos_x = car_car_pos_x - lfw - lrv;
    float new_car_car_pos_y = car_car_pos_y;
    float new_car_pos_x = cos(car_theta)*new_car_car_pos_x - sin(car_theta)*new_car_car_pos_y;
    float new_car_pos_y = -sin(car_theta)*new_car_car_pos_x + sin(car_theta)*new_car_car_pos_y;
    double dx = wayPt.x - new_car_pos_x;
    double dy = wayPt.y - new_car_pos_y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < Lrv)
        return false;
    else if(dist >= Lrv && dist < Lrv*2 )
        return true;
}
geometry_msgs::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
	geometry_msgs::Point odom_path_wayPt;
	
    double carPose_yaw = getYawFromPose(carPose);
    foundForwardPt = false;
    static int BackwardWayPt = 0;
    //int ForwardWayPt = 0;
    if(!goal_reached)
    {
        for(int i =0; i< map_path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;

            try
            {
                tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                odom_path_wayPt = odom_path_pose.pose.position;
                bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                if(_isWayPtAwayFromLfwDist)
                {
					break;
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }

		
		float delta = 0;
		int _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose,delta);	
		
		if(_isForwardWayPt == 3)//正常前进
		{
            Reverse_Drive_Flag = false;
            Reverse_Drive_Flag2 = false;
			BackwardWayPt = 0;
			forwardPt = odom_path_wayPt;
			foundForwardPt = true;	
            odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
            odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);



 		   // ROS_INFO("carPose_yaw=%f,goal=%f,delta = %f",carPose_yaw*rad2deg,rad2deg*atan2(odom_path_wayPt.y-carPose_pos.y,odom_path_wayPt.x-carPose_pos.x),delta*rad2deg);                    
           	
		}
		else
		{
			// ROS_INFO("====Reverse BackwardWayPt %d, Reverse_Drive_Flag = %d !!!!====",BackwardWayPt,Reverse_Drive_Flag);
			// BackwardWayPt++;
			// if(BackwardWayPt > 10)          //10个周期路径位于车体后方或者目标点过偏，需要调头倒车调整姿态
			// {
				// BackwardWayPt = 0;
                if(_isForwardWayPt == 2)//使用倒车控制算法
				{
                    //ROS_INFO("Reverse mode ========");
			        forwardPt = odom_path_wayPt;
                    //====================
                    float car_carPose_pos_x = cos(carPose_yaw)*carPose_pos.x + sin(carPose_yaw)*carPose_pos.y;
                    float car_carPose_pos_y = -sin(carPose_yaw)*carPose_pos.x + cos(carPose_yaw)*carPose_pos.y;
                
                    float new_car_carPose_pos_x = car_carPose_pos_x - lfw - lrv;
                    float new_car_carPose_pos_y = car_carPose_pos_y;

                    float new_carPose_pos_x = cos(carPose_yaw)*new_car_carPose_pos_x - sin(carPose_yaw)*new_car_carPose_pos_y;
                    float new_carPose_pos_y = sin(carPose_yaw)*new_car_carPose_pos_x + cos(carPose_yaw)*new_car_carPose_pos_y;
                    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - new_carPose_pos_x) + sin(carPose_yaw)*(forwardPt.y - new_carPose_pos_y);
                    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - new_carPose_pos_x) + cos(carPose_yaw)*(forwardPt.y - new_carPose_pos_y);

                    //====================    
                    foundForwardPt = false;
                    Reverse_Drive_Flag = false;
                    Reverse_Drive_Flag2 = true;
                }
                else if(_isForwardWayPt == 1)//cmd_vel
                {
                   // ROS_INFO("cmd/vel mode ========");
                    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
                    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);

                    foundForwardPt = false;
                    Reverse_Drive_Flag = true;
                    Reverse_Drive_Flag2 = false;
                }
			// }
		}
    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    points_foward.points.clear();
    line_strip.points.clear();

    if(foundForwardPt || Reverse_Drive_Flag)
    {
        points.points.push_back(carPose_pos);
        points_foward.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(points_foward);


    return odom_car2WayPtVec;
}
geometry_msgs::Point L1Controller::get_odom_car2WayPtVec2(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec2;
	geometry_msgs::Point odom_path_wayPt;
	
    double carPose_yaw = getYawFromPose(carPose);
    foundForwardPt = false;
    static int BackwardWayPt = 0;
    int ForwardWayPt = 0;
    if(!goal_reached)
    {
        for(int i =0; i< map_path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];
            geometry_msgs::PoseStamped odom_path_pose;

            try
            {
                tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);
                odom_path_wayPt = odom_path_pose.pose.position;
                bool _isWayPtAwayFromLrvDist = isWayPtAwayFromLrvDist(odom_path_wayPt,carPose);
                if(_isWayPtAwayFromLrvDist)
                {
					break;
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }

		
		float delta = 0;
		int _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose,delta);	
		
		if(_isForwardWayPt == 3)
		{
            Reverse_Drive_Flag = false;
            Reverse_Drive_Flag2 = false;
			BackwardWayPt = 0;
			forwardPt = odom_path_wayPt;
			foundForwardPt = true;
 		    ROS_INFO("carPose_yaw=%f,goal=%f,delta = %f",carPose_yaw*rad2deg,rad2deg*atan2(odom_path_wayPt.y-carPose_pos.y,odom_path_wayPt.x-carPose_pos.x),delta*rad2deg);                    
           	
		}
		else
		{
			// ROS_INFO("====Reverse BackwardWayPt %d, Reverse_Drive_Flag = %d !!!!====",BackwardWayPt,Reverse_Drive_Flag);
			//BackwardWayPt++;
			 if(BackwardWayPt > 10)        //10个周期路径位于车体后方或者目标点过偏，需要调头倒车调整姿态
		     {
			 	BackwardWayPt = 0;
                if(_isForwardWayPt == 2)
				{
                    foundForwardPt = false;
                    Reverse_Drive_Flag = false;
                    Reverse_Drive_Flag2 = true;
                }
                else if(_isForwardWayPt == 1)//cmd_vel
                {
                    foundForwardPt = false;
                    Reverse_Drive_Flag = true;
                    Reverse_Drive_Flag2 = false;
                }
			}
		}
    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();

    if(foundForwardPt && !goal_reached)
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }
    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    float car_carPose_pos_x = cos(carPose_yaw)*carPose_pos.x + sin(carPose_yaw)*carPose_pos.y;
    float car_carPose_pos_y = -sin(carPose_yaw)*carPose_pos.x + cos(carPose_yaw)*carPose_pos.y;
   
    float new_car_carPose_pos_x = car_carPose_pos_x - lfw - lrv;
    float new_car_carPose_pos_y = car_carPose_pos_y;

    float new_carPose_pos_x = cos(carPose_yaw)*new_car_carPose_pos_x - sin(carPose_yaw)*new_car_carPose_pos_y;
    float new_carPose_pos_y = sin(carPose_yaw)*new_car_carPose_pos_x + cos(carPose_yaw)*new_car_carPose_pos_y;
    odom_car2WayPtVec2.x = cos(carPose_yaw)*(forwardPt.x - new_carPose_pos_x) + sin(carPose_yaw)*(forwardPt.y - new_carPose_pos_y);
    odom_car2WayPtVec2.y = -sin(carPose_yaw)*(forwardPt.x - new_carPose_pos_x) + cos(carPose_yaw)*(forwardPt.y - new_carPose_pos_y);
    return odom_car2WayPtVec2;
}
double L1Controller::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
    return eta;
}
double L1Controller::getEta2(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec2 = get_odom_car2WayPtVec2(carPose);

    double eta2 = atan2(odom_car2WayPtVec2.y,odom_car2WayPtVec2.x);
    return eta2;
}////xuwanshang////////////////////////
double L1Controller::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);

    return dist2goal;
}
double L1Controller::getL1Distance(const double& _Vcmd)
{
    double L1 = 0;
    if(_Vcmd < 1.34)
        L1 = 1.5;
    else if(_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd*2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}
double L1Controller::getSteeringAngle(double eta)
{
    return -atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)));
}
double L1Controller::getSteeringAngle2(double eta2)
{
    return -atan2((L*sin(eta2)),(Lrv/2+lrv*cos(eta2)));
}
double L1Controller::getGasInput(const float& current_v)
{
    double u = (Vcmd - current_v)*Gas_gain;
    //ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    return u;
}
void L1Controller::goal_region_receive(const std_msgs::Int8& receive_goal_region)
{
    goal_region = receive_goal_region;
     ROS_INFO("Goal Region recived : %d!",goal_region.data);

}
void L1Controller::goalReachingCB(const ros::TimerEvent&)
{

    if(goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        if(car2goal_dist < goalRadius)
        {
            goal_reached = true;
            goal_received = false;
            steer_eta_integral = 0;                         // !!!!!!!!!!!!!
            ROS_INFO("Goal Reached !");

            goal_notify_pub.publish(goal_region);
        }
    }
}
void L1Controller::controlLoopCB(const ros::TimerEvent&)
{
    geometry_msgs::Pose carPose = odom.pose.pose;
	std_msgs::Float64 msgRearR,msgRearL,msgSteerR,msgSteerL;

    if(goal_received && map_path.poses.size()>1)
    {
        /*Estimate Steering Angle*/

        std_msgs::Float64 pid_data;
        double eta = getEta(carPose);
        double eta2 = eta;//getEta2(carPose);
        if(foundForwardPt)
        {
           // ROS_INFO("   Foward Drive [||||||========------->>>>>>>"); 
            Reverse_Drive_Flag = 0;
            double steer_angle = -1*getSteeringAngle(eta);

            steer_eta_integral += eta;
            //ROS_INFO("Steering Angle is = %.2f",steer_angle);

            // if(fabs(steer_angle) > 0.2)
            //     steer_angle *= 2.1;
			steer_angle = kp * steer_angle + ki * steer_eta_integral + kd * (eta - steer_eta_last);
           
            // pid_data.data = steer_angle;
            // PID_pub.publish(pid_data);

            steer_eta_last = eta;

            double T_rear = 0.030;
            double T_front = 0.030;
            double L = 0.335;

            double r = L/fabs(tan(steer_angle));

            double rL_rear = r-(sign(steer_angle)*(T_rear/2.0));
            double rR_rear = r+(sign(steer_angle)*(T_rear/2.0));
            double rL_front = r-(sign(steer_angle)*(T_front/2.0));
            double rR_front = r+(sign(steer_angle)*(T_front/2.0));

            msgRearR.data = Vcmd * 20;
            pub_vel_right_rear_wheel.publish(msgRearR);
            pub_vel_right_front_wheel.publish(msgRearR);

            msgRearL.data = Vcmd * 20;
            pub_vel_left_rear_wheel.publish(msgRearL);
            pub_vel_left_front_wheel.publish(msgRearL);

            msgSteerL.data = atan2(L,rL_front)*sign(steer_angle);
            pub_pos_left_steering_hinge.publish(msgSteerL);

            msgSteerR.data = atan2(L,rR_front)*sign(steer_angle);
            pub_pos_right_steering_hinge.publish(msgSteerR);

            // ROS_INFO("steer_angle=%.2f",steer_angle);

        }   //end of if(foundForwardPt)
        //==   BackWard Drive   ==//
        if(Reverse_Drive_Flag)
        {
            //  ROS_INFO("Reverse_Drive_Control Send");
            //ROS_INFO("Reverse_Drive <<<<<<<-------========||||||]");
            steer_eta_integral = 0;                         // !!!!!!!!!!!!!

            double r = 0;
            if (fabs(Reverse_angle) <1e-5)
                r = 1.0e5;
            else    
                r = L/20*fabs(tan(Reverse_angle)); 

            // turn radius
            double rL_rear = r-(sign(Reverse_angle)*(T_rear/2.0));
            double rR_rear = r+(sign(Reverse_angle)*(T_rear/2.0));
            double rL_front = r-(sign(Reverse_angle)*(T_front/2.0));
            double rR_front = r+(sign(Reverse_angle)*(T_front/2.0));


            msgSteerL.data = atan2(L,rL_front)*sign(Reverse_angle);
            // pub_pos_left_steering_hinge.publish(msgSteerL);

            msgSteerR.data = atan2(L,rR_front)*sign(Reverse_angle);
            // pub_pos_right_steering_hinge.publish(msgSteerR);


            msgRearR.data = sign(Reverse_speed) * 5; //VELOCITY * rR_rear/r;
            // pub_vel_right_rear_wheel.publish(msgRearR);
            // pub_vel_right_front_wheel.publish(msgRearR);

            msgRearL.data = sign(Reverse_speed) * 5; //VELOCITY * rL_rear/r;
            // pub_vel_left_rear_wheel.publish(msgRearL);
            // pub_vel_left_front_wheel.publish(msgRearL);
        }
       //end of if(goal_received)
        if(Reverse_Drive_Flag2)
        {
            //ROS_INFO("Reverse mode======"); 
            double steer_angle = -1*getSteeringAngle2(eta2);

            steer_eta_integral2 += eta2;
            //ROS_INFO("Steering Angle is = %.2f",steer_angle);

            // if(fabs(steer_angle) > 0.2)
            //     steer_angle *= 2.1;
			steer_angle = kp * steer_angle + ki * steer_eta_integral2 + kd * (eta2 - steer_eta_last2);
            //ROS_INFO("steer_angle=%.2f",steer_angle);
            // pid_data.data = steer_angle;
            // PID_pub.publish(pid_data);

            steer_eta_last2 = eta2;

            double T_rear = 0.030;
            double T_front = 0.030;
            double L = 0.335;

            double r = L/fabs(tan(steer_angle));

            double rL_rear = r-(sign(steer_angle)*(T_rear/2.0));
            double rR_rear = r+(sign(steer_angle)*(T_rear/2.0));
            double rL_front = r-(sign(steer_angle)*(T_front/2.0));
            double rR_front = r+(sign(steer_angle)*(T_front/2.0));

            // msgRearR.data = -1 * Vcmd * 20;
            // pub_vel_right_rear_wheel.publish(msgRearR);
            // pub_vel_right_front_wheel.publish(msgRearR);

            // msgRearL.data = -1 * Vcmd * 20;
            // pub_vel_left_rear_wheel.publish(msgRearL);
            // pub_vel_left_front_wheel.publish(msgRearL);

            // msgSteerL.data = atan2(L,rL_front)*sign(steer_angle);
            // pub_pos_left_steering_hinge.publish(msgSteerL);

            // msgSteerR.data = atan2(L,rR_front)*sign(steer_angle);
            // pub_pos_right_steering_hinge.publish(msgSteerR);

            // ROS_INFO("steer_angle=%.2f",steer_angle);

        }
    }
	else
	{
            ROS_INFO("Dummy State!!");
            msgRearR.data = 0;
            pub_vel_right_rear_wheel.publish(msgRearR);
            pub_vel_right_front_wheel.publish(msgRearR);

            msgRearL.data = 0;
            pub_vel_left_rear_wheel.publish(msgRearL);
            pub_vel_left_front_wheel.publish(msgRearL);
	}

}
void L1Controller::cmd_vel_receive(const geometry_msgs::Twist& data)
{
    // ROS_INFO("================Reverse cmd_vel received %d !!!!=========================",Reverse_Drive_Flag);

    if(Reverse_Drive_Flag)
    {
		Reverse_angle = data.angular.z;
		Reverse_speed = data.linear.x;

    }
}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "L1Controller_v2");
    L1Controller controller;
    ros::spin();
    return 0;
}
