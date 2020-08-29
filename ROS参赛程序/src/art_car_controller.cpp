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
        ros::Subscriber odom_sub, path_sub, goal_sub,goal_region_sub,final_region_sub;
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
    if (pn.getParam("stanley_k", k))
    {
        //pn.param("stanley_v", v, 0.005);					//只用一个k调节参数
        ROS_WARN("Stanley parameters stanley_k Loaded ");
    }
    else
    {
        ROS_ERROR("Failed to load Stanley parameters");
        return;
    }	
	///////////////////////////////////////////////
    if (pn.hasParam("controller_freq"))
		pn.getParam("controller_freq", controller_freq);
	else
    {
        ROS_ERROR("Parameter controller_freq doesn't exsist!");
        return ;
    }	
	///////////////////////////////////////////////
	
	/**********************/

    odom_sub = n_.subscribe("odometry/filtered", 1, &StanleyController::odomCB, this);
    path_sub = n_.subscribe("/move_base/GlobalPlanner/plan", 1, &StanleyController::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &StanleyController::goalCB, this);
    goal_region_sub = n_.subscribe("/goal_region",1,&StanleyController::goal_region_receive,this);

    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 1);
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);
    goal_notify_pub = n_.advertise<std_msgs::Int8>("/art_car_controller/goal_finish",1);

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &StanleyController::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((0.5)/10), &StanleyController::goalReachingCB, this); // Duration(0.05) -> 20Hz

    //Init variables
    initMarker();
    //Show info
    ROS_INFO("[param] stanley_k: %f", k);
    //ROS_INFO("[param] stanley_v: %f", v);
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
				distance_min = distance_now;	//应该换算为车头到目标点距离
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

	if(map_path.poses[index].pose.position.y >  carPose.position.y)
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
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos,double& );
        double getYawFromPose(const geometry_msgs::Pose& carPose);
        double getEta(const geometry_msgs::Pose& carPose);
        double getCar2GoalDist();
        double getL1Distance(const double& _Vcmd);
        double getSteeringAngle(double eta);
        double getGasInput(const float& current_v);
        geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);

    private:
        ros::NodeHandle n_;
        ros::Subscriber odom_sub, path_sub, goal_sub,goal_region_sub, move_base_cmdvel_sub,final_region_sub;
        ros::Publisher pub_vel_angle,PID_pub,goal_notify_pub,pub_, marker_pub,pub_vel_left_rear_wheel,pub_vel_right_rear_wheel,pub_vel_left_front_wheel,pub_vel_right_front_wheel,pub_pos_left_steering_hinge,pub_pos_right_steering_hinge;
        ros::Timer timer1, timer2;
        tf::TransformListener tf_listener;

        visualization_msgs::Marker points, line_strip, goal_circle;
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Point odom_goal_pos;
        nav_msgs::Odometry odom;
        nav_msgs::Path map_path;
        std_msgs::Int8 goal_region;
        double Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v,kp,ki,kd;
        double Gas_gain, baseAngle, Angle_gain, goalRadius;
        double baseSpeed,controller_freq;
        bool foundForwardPt, goal_received, goal_reached;
        double steer_eta_last ,steer_eta_integral;
        bool Reverse_Drive_Flag,Turn_Around_Flag;
        double T_front;
        double T_rear ;
        double L;
        double Reverse_angle;
        double Reverse_speed;
        double view_filed;
        double speed_out;
        double car_car2wayPt_x_thres;

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void goalReachingCB(const ros::TimerEvent&);
        void controlLoopCB(const ros::TimerEvent&);
        void goal_region_receive(const std_msgs::Int8& receive_goal_region);
        void cmd_vel_receive(const geometry_msgs::Twist& );
        void on_final_region(const std_msgs::Int8& );

}; // end of class


L1Controller::L1Controller()
{
    //Private parameters handler
    ros::NodeHandle pn("art_car_controller");

    //Car parameter
    pn.param("L", L, 0.35);
    // pn.param("Vcmd", Vcmd, 1.0);

    // pn.param("Lrv", Lrv, 1.0);
    pn.param("lfw", lfw, 0.1675);
    pn.param("lrv", lrv, 0.1675);
	
	/////////////////////////////////////
    //Controller parameter    
	if (pn.hasParam("kp"))
		pn.getParam("kp", kp);
	else
    {
        ROS_ERROR("Parameter kp doesn't exsist!");
        return ;
    }
	///////////////////////////////////////////
    if (pn.hasParam("ki"))
		pn.getParam("ki", ki);
	else
    {
        ROS_ERROR("Parameter ki doesn't exsist!");
        return ;
    }		
	/////////////////////////////////////////
    if (pn.hasParam("kd"))
		pn.getParam("kd", kd);
	else
    {
        ROS_ERROR("Parameter kd doesn't exsist!");
        return ;
    }		
	////////////////////////////////////////////
    if (pn.hasParam("view_filed"))
		pn.getParam("view_filed", view_filed);
	else
    {
        ROS_ERROR("Parameter view_filed doesn't exsist!");
        return ;
    }		
	//////////////////////////////////////////
    if (pn.hasParam("speed_out"))
		pn.getParam("speed_out", speed_out);
	else
    {
        ROS_ERROR("Parameter speed_out doesn't exsist!");
        return ;
    }	
	///////////////////////////////////////////////
    if (pn.hasParam("controller_freq"))
		pn.getParam("controller_freq", controller_freq);
	else
    {
        ROS_ERROR("Parameter controller_freq doesn't exsist!");
        return ;
    }	
	///////////////////////////////////////////////
    if (pn.hasParam("baseSpeed"))
		pn.getParam("baseSpeed", baseSpeed);
	else
    {
        ROS_ERROR("Parameter baseSpeed doesn't exsist!");
        return ;
    }	
	///////////////////////////////////////////////
    if (pn.hasParam("baseAngle"))
		pn.getParam("baseAngle", baseAngle);
	else
    {
        ROS_ERROR("Parameter baseAngle doesn't exsist!");
        return ;
    }	
    /////////////////////////////////////////////
    if (pn.hasParam("car_car2wayPt_x_thres"))
		pn.getParam("car_car2wayPt_x_thres", car_car2wayPt_x_thres);
	else
    {
        ROS_ERROR("Parameter car_car2wayPt_x_thres doesn't exsist!");
        return ;
    }	

    ////////////////////////////////////////////////////	
    {
    //Publishers and Subscribers
    odom_sub = n_.subscribe("odometry/filtered", 1, &L1Controller::odomCB, this);
    path_sub = n_.subscribe("/move_base/GlobalPlanner/plan", 1, &L1Controller::pathCB, this);
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);
    goal_region_sub = n_.subscribe("/goal_region",1,&L1Controller::goal_region_receive,this);
    move_base_cmdvel_sub = n_.subscribe("/cmd_vel",1,&L1Controller::cmd_vel_receive,this);
	final_region_sub = n_.subscribe("/final_region",1,&L1Controller::on_final_region,this);

    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 1);
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);
    goal_notify_pub = n_.advertise<std_msgs::Int8>("/art_car_controller/goal_finish",1);
    PID_pub = n_.advertise<std_msgs::Float64>("err",1);

	pub_vel_angle = n_.advertise<geometry_msgs::Twist>("/car/cmd_vel", 1);


    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &L1Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz
    timer2 = n_.createTimer(ros::Duration((1.0)/10), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz

    //Init variables
    Lrv = Lfw = getL1Distance(Vcmd);
    goalRadius = 1.5;

    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;

    steer_eta_last = 0;
    steer_eta_integral = 0;
    Reverse_Drive_Flag = Turn_Around_Flag = false;
    T_front = 0.3;
    T_rear = 0.3;
    L = 0.35;
	lfw = 0.17;
    Reverse_angle = 0;
    Reverse_speed = 0;

    //Show info
    ROS_INFO("------------------------");
    ROS_INFO("[param] Lfw: %f", Lfw);//前进参数

    ROS_INFO("[param] L: %f", L);

    ROS_INFO("[param] Lrv: %f", Lrv);//倒车参数
    ROS_INFO("[param] lrv: %f", lrv);
    ROS_INFO("[param] lfw: %f", lfw);	
    
    ROS_WARN("------------------------");
    
    ROS_WARN("[param] kp: %f", kp);
    ROS_WARN("[param] ki: %f", ki); 
    ROS_WARN("[param] kd: %f", kd);

    ROS_WARN("[param] controller_freq: %f", controller_freq);
    ROS_WARN("[param] view_filed: %f", view_filed);

    ROS_WARN("[param] baseAngle: %f", baseAngle);
    ROS_WARN("[param] baseSpeed: %f", baseSpeed);
    ROS_WARN("[param] speed_out: %f", speed_out);

    ROS_WARN("------------------------");
    //Visualization Marker Settings
    initMarker();
    }
}
void L1Controller::initMarker()
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

    goal_circle.scale.x = goalRadius;
    goal_circle.scale.y = goalRadius;
    goal_circle.scale.z = 0.1;

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
void L1Controller::on_final_region(const std_msgs::Int8& )
{
    this->goalRadius = 0.5;
    ROS_WARN("goalRadius is set 0.5,Moving to Last Goal.............");
}
void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    // ROS_INFO("odomCB");
    odom = *odomMsg;
}
void L1Controller::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    // ROS_INFO("pathCB");
    map_path = *pathMsg;
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
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;
	
	delta = goal_theta - car_theta;
	if(fabs(delta) > PI)
		delta = PI*2 - delta*sign(delta);
	

    // ROS_INFO("car_car2wayPt_x=%f",car_car2wayPt_x);
    //ROS_INFO("carPose_yaw=%f,goal=%f,delta = %f",car_theta*rad2deg,rad2deg*goal_theta,delta*rad2deg);                    

    //if(car_car2wayPt_x  > car_car2wayPt_x_thres && fabs(delta*rad2deg) < view_filed) /*is Forward WayPt and in the turn angle?*/
        return 3;//路径在车辆前方，正常前进

    // if(car_car2wayPt_x  < 0 && fabs(car_car2wayPt_x) > car_car2wayPt_x_thres)
    //     return 2;//路径在车辆后方，使用倒车控制算法

    //return 1;       //路径与车头水平：car_car2wayPt_x>0，但是不在视角范围内；此时订阅cmd_vel，， 调头
}
bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos ,double& ladder)
{
    /*double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);
	ladder += dist;
    if(ladder < Lfw)
        return false;
    else 
		if(ladder >= Lfw && ladder < Lfw*2 )
			return true;
		else
			return false;*/
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < Lfw)
        return false;
    else if(dist >= Lfw && dist < Lfw*2 )
        return true;
}
geometry_msgs::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
	geometry_msgs::Point odom_path_wayPt;
	
    static int BackwardWayPt_count = 0;
    static int ForwardWayPt_count = 0;    
    double ladder = 0;
	
	double carPose_yaw = getYawFromPose(carPose);
	
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
                bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos,ladder);
				// ROS_INFO("ladder=%f",ladder);

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
            // ROS_INFO("_isForwardWayPt=%d",_isForwardWayPt);
		if(_isForwardWayPt == 3) //前瞻点有效,正常前进
		{
            if(!foundForwardPt)      //上一周期为不为正常前进状态
            {
                ForwardWayPt_count++;
 
                if(ForwardWayPt_count > -1)//20)            //连续20个周期满足正常前进条件
                {
                    ForwardWayPt_count = 0;
                    BackwardWayPt_count = 0;
                
                    forwardPt = odom_path_wayPt;
                    foundForwardPt = true;		
                    Reverse_Drive_Flag = false;			
                    Turn_Around_Flag = 	false;
                    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
                    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
                    
                //    ROS_INFO("Foward Drive [||||||========------->>>>>>>"); 
 		  //          ROS_INFO("carPose_yaw=%f,goal=%f,delta = %f",carPose_yaw*rad2deg,rad2deg*atan2(odom_path_wayPt.y-carPose_pos.y,odom_path_wayPt.x-carPose_pos.x),delta*rad2deg);                    
                }
				//else 状态保持为上一周期状态
            }
            else //上一周期为正常前进状态
            {
                ForwardWayPt_count = 0;
                BackwardWayPt_count = 0;
            
                forwardPt = odom_path_wayPt;
                foundForwardPt = true;		
                Reverse_Drive_Flag = false;	
                Turn_Around_Flag = 	false;

                odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
                odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);

              //  ROS_INFO("Foward Drive [||||||========------->>>>>>>"); 
              //  ROS_INFO("carPose_yaw=%f,goal=%f,delta = %f",carPose_yaw*rad2deg,rad2deg*atan2(odom_path_wayPt.y-carPose_pos.y,odom_path_wayPt.x-carPose_pos.x),delta*rad2deg);                    

            }

            
		}
		else 
        if(_isForwardWayPt == 1)	//前瞻点无效，与车头夹角太大或者找到远端，需要订阅cmd_vel来调头
		{
			// ROS_INFO("====Reverse BackwardWayPt_count %d, Reverse_Drive_Flag = %d !!!!====",BackwardWayPt_count,Reverse_Drive_Flag);
            if(!Turn_Around_Flag)            //上一个周期不为调头状态
            {
                BackwardWayPt_count++;
                if(BackwardWayPt_count > -1)//10)          //连续10个周期如果位于车体后方或者目标点过偏，需要调头倒车调整姿态
                {
                    BackwardWayPt_count = 0;
                    ForwardWayPt_count = 0;
                    
                    Turn_Around_Flag = true;
                    Reverse_Drive_Flag = false;
					foundForwardPt = false;
                    ROS_ERROR("Turn Around mode!!!!!!!!!!!!!!");
                }
				//else 状态保持为上一周期状态				
            }
            else		 //上一个周期为调头状态
            {
                BackwardWayPt_count = 0;
                ForwardWayPt_count = 0;
                
                Turn_Around_Flag = true;
                Reverse_Drive_Flag = false;
                foundForwardPt = false;
                ROS_ERROR("Turn Around mode!!!!!!!!!!!!!!");
            }
            
		}
        else// _isForwardWayPt = 2 : 使用倒车控制算法
        {
            ROS_ERROR("Reverse_Drive <<<<<<<-------========||||||]");
            ForwardWayPt_count = 0;
            BackwardWayPt_count = 0;

            Reverse_Drive_Flag = true;
            Turn_Around_Flag = false;
            foundForwardPt = false;
            
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

        }
        

    }// end of if(!goal_reached)   
    else // goal_reached = true
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
		Reverse_Drive_Flag = false;
        Turn_Around_Flag = false;
        ROS_INFO("goal REACHED!---------------------------------------------------------");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    points.points.clear();
    line_strip.points.clear();

    points.points.push_back(carPose_pos);
    points.points.push_back(odom_path_wayPt);//forwardPt);

    if(foundForwardPt || Reverse_Drive_Flag)
    {								   
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }
    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    return odom_car2WayPtVec;
}
double L1Controller::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);

    double eta = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
    return eta;
}
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
        L1 = 0.6;
    else if(_Vcmd > 1.34 && _Vcmd < 5.36)
        L1 = _Vcmd*2.24 / 3.0;
    else
        L1 = 12 / 3.0;
    return L1;
}
double L1Controller::getSteeringAngle(double eta)
{
    return atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)));
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
    ROS_ERROR("Goal Region recived : %d!",goal_region.data);

}
void L1Controller::goalReachingCB(const ros::TimerEvent&)
{

    if(goal_received)
    {
        double car2goal_dist = getCar2GoalDist();
        if(car2goal_dist < goalRadius )
        {
            goal_reached = true;
            goal_received = false;
            // steer_eta_integral = 0;                         // !!!!!!!!!!!!!
            ROS_INFO("Goal Reached !");

            goal_notify_pub.publish(goal_region);
        }
    }
}
void L1Controller::controlLoopCB(const ros::TimerEvent&)
{
    geometry_msgs::Pose carPose = odom.pose.pose;
	geometry_msgs::Twist control_out;
    double steer_pwm,motor_pwm,steer_angle_out;
	double steer_angle ;                
    if(goal_reached)
    {
        //ROS_INFO("Goal Reached !");
        
 		control_out.angular.z = 1500;
		control_out.linear.x = 1500;
    	pub_vel_angle.publish(control_out);		
        goal_notify_pub.publish(goal_region);
        //ROS_INFO("pub goal_region%d",goal_region);
    	return;
    }
    // ROS_INFO("map_path.poses.size =%d",map_path.poses.size());
    if(goal_received && map_path.poses.size()>20)   //后面是为了为防止路径还没有全部计算出时导致误判为调头
    {
        /*Estimate Steering Angle*/
        std_msgs::Float64 pid_data;
        double eta = getEta(carPose);			//rad
        if(!Turn_Around_Flag) //非调头状态
        {

            steer_angle = -1*getSteeringAngle(eta)*rad2deg;		//rad===>degree

            //steer_eta_integral += (eta*rad2deg);
            //ROS_INFO("Steering Angle=%.2f",steer_angle*rad2deg);

            // if(fabs(steer_angle) > 0.2)
            //     steer_angle *= 2.1;
			steer_angle_out = baseAngle - ( -1 * kp * steer_angle + ki * steer_eta_integral + kd * (eta*rad2deg - steer_eta_last*rad2deg));//虚拟转角：steer_angle为正数，右手坐标系下左转，PWM应减小输出;;;;steer_angle为负数，右手坐标系下右转，PWM应增大输出, the unit is    DEGREE!!!!!!!!!!!!!!!!
           
            pid_data.data = steer_angle_out;
            PID_pub.publish(pid_data);

            steer_eta_last = eta;		//rad

            steer_pwm  = steer_angle_out ;//舵机已有闭环，直接可以跟踪指令角？？？

            if(foundForwardPt)  //前进
            {
                motor_pwm  = baseSpeed + speed_out;
		        if(motor_pwm  > 2000)           //输出限幅
		            motor_pwm  = 2000;
		        if(motor_pwm  < 1500)       //限制为前进
                	motor_pwm  = 1500;  

			}
            else
            {
                motor_pwm = baseSpeed - speed_out - 50;
                if(motor_pwm  < 1000)           //输出限幅，最大后退速度
                    motor_pwm  = 1000;
                if(motor_pwm  > 1500)       //限制为后退
                    motor_pwm  = 1500;  


            }
            //ROS_INFO("Steering Angle=%.2f", steer_angle*rad2deg );            
        }   //end of if(foundForwardPt)
        //==   BackWard Drive   ==//
        else if(Turn_Around_Flag)
        {          
            Reverse_angle  = PI / 2 + Reverse_angle;

            steer_pwm = 2500 - 2000 * Reverse_angle / PI;//Reverse_angle 有正负之分
            if(Reverse_speed < 0)       //倒车
            {
                motor_pwm = baseSpeed - speed_out - 50;

                if(motor_pwm  < 1000)           //输出限幅，，订阅局部路径时的最大后退速度
                    motor_pwm  = 1000;
                if(motor_pwm  > 1500)       //限制为后退
                    motor_pwm  = 1500;
            }
            else        //前进
            {
                motor_pwm = baseSpeed + speed_out;

                if(motor_pwm  < 1500)           //限制为前进
                    motor_pwm  = 1500;
                if(motor_pwm  > 2000)       //输出限幅，订阅局部路径时的最大前进速度
                    motor_pwm  = 2000;
            }

           //ROS_INFO("Reverse_angle =%.2f,steer_pwm =%f,motor_pwm=%f", Reverse_angle*rad2deg ,steer_pwm,motor_pwm);

        }//end of else if(Turn_Around_Flag)
        // else if(Reverse_Drive_Flag)
        // {
            
        // }
        //////////////////////////////////////////////////////
    }   //end of if(goal_received)
	else		//goal_received = 0 或者 接收到终点，但是全局路径还未规划出
	{
        steer_pwm = 1500;
        motor_pwm = 1500;
        ROS_WARN("Dummy State!!!!!!!!!!!!");
	}

    if(steer_pwm  > 2000)           //舵机输出限幅
        steer_pwm  = 2000;
    if(steer_pwm  < 800)
        steer_pwm  = 800;

    //ROS_INFO("steer_pwm =%f,motor_pwm=%f",steer_pwm,motor_pwm);
    //ROS_INFO("-Steering Angle=%.2f,steer_pwm =%f,motor_pwm=%f", -steer_angle,steer_pwm,motor_pwm);     
    control_out.angular.z = steer_pwm; 
    control_out.linear.x = motor_pwm;
   	pub_vel_angle.publish(control_out);
}
void L1Controller::cmd_vel_receive(const geometry_msgs::Twist& data)
{
    // ROS_INFO("================Reverse cmd_vel received %d !!!!=========================",Reverse_Drive_Flag);

    //if(Turn_Around_Flag)
    //{
		Reverse_angle = data.angular.z;
		Reverse_speed = data.linear.x;

    //}
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
