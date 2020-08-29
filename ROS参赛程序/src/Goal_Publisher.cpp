#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <string.h>
#include <std_msgs/Int8.h>

 
#include <vector>
#include <map>
#include <iostream>

using namespace std;

int goal_index[30];
int goal_index_out[30];
int goal_number;
int head_angle,head_region;	

double goal_array[11][2]={
	//{0.765,-0.267},//1
	{-1.53,-0.158},//1======start point
	{5.42,-0.188},//2
	{10.2,-1.2},//3
	{15.5,-0.501},//4
	{15.0,-5.17},//5
	{14.6,-9.06},//6
	{8.35,-9.25},//7
	{3.64,-8.92},//8
	{1.15,-8.88},//9
	{6.97,-4.97},//10
	{9.18,-2.48}//11	
};

class Node_Region
{
public:
	Node_Region(int );
	Node_Region(){}
	int region_number;
	int incidence_angle;
	//vector<pair<int, vector<pair<int, int> >  > >  region_info;//[入射角-{（出射区域-出射角），（...），（...）}]，[。。]
	map <int, vector <pair<int, int> > > region_info_map;//[入射角-{（出射区域-出射角），（...），（...）}]，[。。]
	int passed;										//  [0     -{ (vec_outregion2outangle),()}]
};

class OutputRegionInfo
{
public:
	OutputRegionInfo():out_region_count(0){}
	vector<vector<pair<int, int> > > region_list;//所有的输出路径，上限为100个，第一个元素为区域编号，第二个元素为在该区域的转角用于计算下一个前趋
	int out_region_count;
	bool is_goal_list_finished(vector<pair<int, int> >  path);
};
class Goal_Publisher
{
    public:
        Goal_Publisher();
        void initMarker();
		void calculate_order();
		void calculate_order_new();

    private:
        ros::NodeHandle n_;
		int goal_count;
		


        ros::Subscriber L1_controller_v2_sub;
        ros::Publisher goal_pub,marker_pub,goal_region_pub,final_goal_pub;

        void on_goal(const std_msgs::Int8& goal_num);
        void on_time(const ros::TimerEvent& x);
        ros::Timer timer1;
        visualization_msgs::Marker sphere;

		OutputRegionInfo OutputRegion;
		Node_Region node[11];
		geometry_msgs::PoseStamped odom_goal;  
};

//======================================
void Goal_Publisher::calculate_order()
{

}

void Goal_Publisher::calculate_order_new()
{
	map<int, vector<pair<int, int> > > shortest_path_lenth;

	for (int i = 0; i < 11; i++)
	{
		node[i] = Node_Region(i);
	}
	node[head_region - 1].incidence_angle = head_angle; //设置车头朝向方向-度
	// //
	// ////////////////////////////////////////////////
	int loop_index = 0;
	auto p_node = node[head_region - 1];//从起点的指向点开始寻找

	// //*************************************************************
	auto iterator = p_node.region_info_map.find(p_node.incidence_angle);
	for (int i = 0; i < iterator->second.size(); i++)
	{
		//if(iterator->first == iterator->second[i].first)
		//取出所有的前趋区域
		vector<pair<int, int>> vec;
		vec.push_back(iterator->second[i]);
		OutputRegion.region_list.push_back(vec);//region_list存储了所有路径各自的vector
		OutputRegion.out_region_count++;
		//			cout << "foward:" << vec.back().first << endl;
	}//所有路径的第一个区域点填充完成

	int pre_size = OutputRegion.region_list.size();//接下来每个路径i的前趋节点作为当前节点，继续向后找
	for (;;)
	{		
		int pre_size = OutputRegion.region_list.size();
		for (size_t i = 0; i < pre_size; i++)//所有路径，限制路径长度为22
		{
			cout<<endl<<"pre_size = "<<pre_size<<"trying "<<i<<"path points"<<OutputRegion.region_list.at(i).size()<<endl;

			//======某一路径过超过限制路径长度，删除该路径===================
			if (OutputRegion.region_list[i].size()>22)
			{
				OutputRegion.region_list.erase(i + OutputRegion.region_list.begin());
				pre_size --;
				cout<<"after erase:presize="<<pre_size<<endl;
				continue;
			}
			


			for (int j = 0; j < OutputRegion.region_list.at(i).size(); j++)
			{
				cout<< "=>"<<OutputRegion.region_list.at(0).at(j).first;
			}

			if (!OutputRegion.is_goal_list_finished(OutputRegion.region_list[i]))
			{
				p_node = node[OutputRegion.region_list[i].back().first - 1];//每个路径都是一个分支循环
				p_node.incidence_angle = OutputRegion.region_list[i].back().second;
				node[OutputRegion.region_list[i].back().first - 1].passed++;	//标记节点已被遍历一次

				//////////////====第二轮===========
				auto iterator = p_node.region_info_map.find(p_node.incidence_angle);
				// ROS_INFO("iterator->second.size()=%d",iterator->second.size());
				
				if (iterator->second.size() <= 1)	//仅有一个前趋，无需再增加路径分支
				{
					OutputRegion.region_list[i].push_back(iterator->second.back());//直接增加最后一个节点即可
				}
				else
				{
					///其余节点需要将路径扩充后填入================================
					auto vec = OutputRegion.region_list.at(i);//数据拷贝备份
					OutputRegion.region_list[i].push_back(iterator->second.at(0));//原路径，只需直接挂接在后面即可
										
					for (int k = 1; k < iterator->second.size(); k++)//需要存放的次数，总个数减1
					{
						//向扩充的向量填入之前的节点数据
						vec.push_back(iterator->second[k]);	//加入新数据
						OutputRegion.region_list.push_back(vec);

					}
				}
				
			}
			else
			{
				if(OutputRegion.region_list[i].back().first == goal_index[goal_number-1])
				{
					cout <<endl<< "path:";
					for (int j = 0; j < OutputRegion.region_list.at(i).size(); j++)
					{
						cout<< "=>"<<OutputRegion.region_list.at(i).at(j).first;
					}
					cout <<endl;
					
					shortest_path_lenth.insert(make_pair(OutputRegion.region_list[i].size(), OutputRegion.region_list[i]));

					for (int j = 0; j < shortest_path_lenth.begin()->second.size(); j++)
					{
						goal_index_out[j+2] = shortest_path_lenth.begin()->second[j].first;
						//ROS_INFO("-%d-",goal_index_out[j+2]);
					}
					goal_number = shortest_path_lenth.begin()->second.size()+2;//实际要经过的区域数
				}
			}
		}//路径i的第二轮填充完成
		/////// 循环结束判断  /////////////////////////////
		//======================最优路径显示======================
		// ROS_INFO("shortest_path_lenth.size()=%d",shortest_path_lenth.size());
		if (shortest_path_lenth.size() >= 1)//找到满足要求的3条路径，并且已经按照键值排序好
		{
			if(shortest_path_lenth.begin()->second.size() > sizeof(goal_index_out)/sizeof(int))
			{
				ROS_INFO("goal_index_out 开辟过小");
				return;
			}
			//************************************************************
			//======  输出测试	=========//
			ROS_INFO_STREAM("//==== Output Index Test ==");
			ROS_INFO("goal number is %d, ",goal_number);
			for(int i=0;i<goal_number;i++)
			{
				ROS_INFO("%d, ",goal_index_out[i]);
			}
			ROS_INFO_STREAM("//=========================");
			return;
		}
		// 		//////////////////////////////////////////////////////////

	}//end for (; ;)
}

bool OutputRegionInfo::is_goal_list_finished(vector<pair<int, int> >  path)
{
	bool result = true;

	int k = 1;//略去初始点
	while (k < goal_number)
	{
		if (goal_index[k] == head_region)
		{
			//cout << "jmp head"<<endl;
			k++;
			continue;
		}
		
		bool tmp = false;
		for (int j = 0; j < path.size(); j++)
		{
			tmp |= (path.at(j).first == goal_index[k]);
		}
		k++;

		result &= tmp;
	}
	if (!result)//有一个未索引到即放弃本次遍历
	{
		return false;
	}

	return true;
}
Node_Region::Node_Region(int region_num) :passed(0), region_number(0), incidence_angle(0)
{
	region_number = region_num + 1;//区域编号从1开始
	switch (region_number)
	{ 
	case 1:
	{
		//0度入射，对应的出射情况
		{
			vector<pair<int, int> >  vec_outregion2outangle;
			vec_outregion2outangle.push_back(make_pair(2, -90));	//0度入射，仅能从2以-90度出射			
			region_info_map.insert(make_pair(0, vec_outregion2outangle));
		}
		////////////////////////////////////////////////////
		//90度入射，对应的出射情况
		{
			vector<pair<int, int> >  vec_outregion2outangle;
			vec_outregion2outangle.push_back(make_pair(9, 180));//90度入射，仅能从9以180度出射			
			region_info_map.insert(make_pair(90, vec_outregion2outangle));
		}
		break;
	}
	case 2:		//区域2
	{
		//-90度入射，出射区域为4或10
		{
			vector<pair<int, int> >  vec_outregion2outangle;
			vec_outregion2outangle.push_back(make_pair(10, -135));
			vec_outregion2outangle.push_back(make_pair(4, -90));

			region_info_map.insert(make_pair(-90, vec_outregion2outangle));
		}

		////////////////////////////////////////////////////
		//90度入射
		{
			vector<pair<int, int> >  vec_outregion2outangle;
			vec_outregion2outangle.push_back(make_pair(1, 90));

			region_info_map.insert(make_pair(90, vec_outregion2outangle));//90度入射，对应的出射情况
		}
		//45度入射
		{
			vector<pair<int, int> >  vec_outregion2outangle;
			vec_outregion2outangle.push_back(make_pair(1, 90));

			region_info_map.insert(make_pair(45, vec_outregion2outangle));//45度入射，对应的出射情况
		}
		break;
	}
	case 3://区域3
	{
		//-45度入射
		{
			vector<pair<int, int> >  vec_outregion2outangle;
			vec_outregion2outangle.push_back(make_pair(4, -90));

			region_info_map.insert(make_pair(-45, vec_outregion2outangle));//-45度入射，对应的出射情况
		}
		//135度入射
		{
			vector<pair<int, int> >  vec_outregion2outangle;
			vec_outregion2outangle.push_back(make_pair(11, 135));

			region_info_map.insert(make_pair(135, vec_outregion2outangle));//135度入射，对应的出射情况
		}
	}
	break;
	case 4://区域4，两种入射
	{
		//0度入射，区域2、3皆可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(2, 90));
			vec_outregion2outangle.push_back(make_pair(3, 135));

			region_info_map.insert(make_pair(0, vec_outregion2outangle));//0度入射，对应的出射情况
		}
		//-90度入射，区域5可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(5, 180));

			region_info_map.insert(make_pair(-90, vec_outregion2outangle));//-90度入射，对应的出射情况
		}
	}
	break;
	case 5://区域5，三种入射
	{
		//0度入射，区域4、11皆可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(11, 45));
			vec_outregion2outangle.push_back(make_pair(4, 0));

			region_info_map.insert(make_pair(0, vec_outregion2outangle));//0度入射，对应的出射情况
		}
		//-135度入射，区域6可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(6, 180));

			region_info_map.insert(make_pair(-135, vec_outregion2outangle));//0度入射，对应的出射情况
		}
		//180度入射，区域6可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(6, 180));

			region_info_map.insert(make_pair(180, vec_outregion2outangle));//0度入射，对应的出射情况
		}
		break;
	}
	case 6://区域6，两种入射
	{
		//180度入射，区域7可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(7, 90));

			region_info_map.insert(make_pair(180, vec_outregion2outangle));//180度入射，对应的出射情况
		}
		//-90度入射，区域5可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(5, 0));

			region_info_map.insert(make_pair(-90, vec_outregion2outangle));//-90度入射，对应的出射情况
		}
		break;
	}

	case 7://区域7三种入射
	{
		//90度入射，区域8\10可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(8, 90));
			vec_outregion2outangle.push_back(make_pair(10, 45));

			region_info_map.insert(make_pair(90, vec_outregion2outangle));//90度入射，对应的出射情况
		}
		//-90度入射，区域6可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(6, -90));
			//=============  测试用 ，增加一个属性  ====================
			//vec_outregion2outangle.push_back(make_pair(1, -90));

			//==========================================================

			region_info_map.insert(make_pair(-90, vec_outregion2outangle));//-90度入射，对应的出射情况
		}
		//-135度入射，区域6可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(6, -90));

			region_info_map.insert(make_pair(-135, vec_outregion2outangle));//-135度入射，对应的出射情况
		}
		break;
	}
	case 8://区域8 ，三种入射情况
	{
		//90度入射，区域9可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(9, 90));

			region_info_map.insert(make_pair(90, vec_outregion2outangle));//90度入射，对应的出射情况
		}
		//-90度入射，区域7、10可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(7, -90));
			vec_outregion2outangle.push_back(make_pair(10, -45));

			region_info_map.insert(make_pair(-90, vec_outregion2outangle));//-90度入射，对应的出射情况
		}
		//135度入射，区域9可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(9, 90));

			region_info_map.insert(make_pair(135, vec_outregion2outangle));//135度入射，对应的出射情况
		}

		break;
	}
	case 9://区域9，两种入射情况
	{
		{   //90度入射，区域1可达
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(1, 0));//区域号--到达区域后的方向角

			region_info_map.insert(make_pair(90, vec_outregion2outangle));//-45度入射，对应的出射情况
		}
		{   //180度入射，区域8可达
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(8, -90));//区域号--到达区域后的方向角

			region_info_map.insert(make_pair(180, vec_outregion2outangle));//-45度入射，对应的出射情况
		}
		break;
	}
	case 10://区域10，四种入射情况
	{
		//-45度入射，区域2、11可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(11, -45));
			vec_outregion2outangle.push_back(make_pair(2, 45));

			region_info_map.insert(make_pair(-45, vec_outregion2outangle));//-45度入射，对应的出射情况
		}
		//45度入射，区域2、11可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(2, 45));
			vec_outregion2outangle.push_back(make_pair(11, -45));

			region_info_map.insert(make_pair(45, vec_outregion2outangle));//45度入射，对应的出射情况
		}
		//135度入射，区域7,8可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(7, -135));
			vec_outregion2outangle.push_back(make_pair(8, 135));
			//=============  测试用 ，增加一个属性  ====================
			//vec_outregion2outangle.push_back(make_pair(5, -135));

			//==========================================================

			region_info_map.insert(make_pair(135, vec_outregion2outangle));//135度入射，对应的出射情况
		}
		//-135度入射，区域7，区域8可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(7, -135));
			vec_outregion2outangle.push_back(make_pair(8, 135));

			region_info_map.insert(make_pair(-135, vec_outregion2outangle));//-135度入射，对应的出射情况
		}
		break;
	}
	case 11://区域11,三种入射情况
	{
		//-45度入射，区域3可达
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(3, -45));

			region_info_map.insert(make_pair(-45, vec_outregion2outangle));//-45度入射，对应的出射情况
		}
		//45度入射，区域======> Angle Too Large!!!
		{
			 vector<pair<int,int> >  vec_outregion2outangle;

			 vec_outregion2outangle.push_back(make_pair(10,135));
			 
			 vec_outregion2outangle.push_back(make_pair(3,-45));

			 region_info_map.insert(make_pair(45,vec_outregion2outangle));//45度入射，对应的出射情况
		}
		//135度入射，区域10可达,，，，区域5的达到角太大。。。 
		{
			vector<pair<int, int> >  vec_outregion2outangle;

			vec_outregion2outangle.push_back(make_pair(10, 135));
			//=============  测试用 ，增加一个属性  ====================
			//vec_outregion2outangle.push_back(make_pair(7, -90));

			//==========================================================
			region_info_map.insert(make_pair(135, vec_outregion2outangle));//135度入射，对应的出射情况
		}
		break;
	}
	}
}

//======================================
void Goal_Publisher::initMarker()
{
    sphere.header.frame_id = "map";
    sphere.ns = "Markers";
    sphere.action = visualization_msgs::Marker::ADD;
    sphere.pose.orientation.w = 1.0;
    sphere.id = 0;

    sphere.type = visualization_msgs::Marker::SPHERE;

    sphere.scale.x = 0.35;
    sphere.scale.y = 0.35;
    sphere.scale.z = 0.35;

    // sphere are RED
    sphere.color.r = 1.0;
    sphere.color.g = 0.0;
    sphere.color.b = 0.0;
    sphere.color.a = 0.5;
}

Goal_Publisher::Goal_Publisher()
{
	/*
	add_executable(Goal_Publisher src/Goal_Publisher.cpp)
	target_link_libraries(Goal_Publisher ${catkin_LIBRARIES})
	add_dependencies(Goal_Publisher findLine_tutorials_generate_messages_cpp)

	
	launch文件格式：
	<rosparam command="load" file="$(find racecar_gazebo)/config/goal_list.yaml" />
	
	<node pkg="racecar_gazebo" type="Goal_Publisher" name="Goal_Publisher" />
	
	*/
    ros::NodeHandle n_("Goal_Publisher");
    if (n_.getParam("goal_number", goal_number))
    {
        ROS_INFO_STREAM("Loaded goal_number" << ": " << goal_number);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load goal_number");
        n_.shutdown();
    }	

    if (n_.getParam("head_angle", head_angle))
    {
        ROS_INFO_STREAM("Loaded head_angle" << ": " << head_angle);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load head_angle");
        n_.shutdown();
    }	

    if (n_.getParam("head_region", head_region))
    {
        ROS_INFO_STREAM("Loaded head_region" << ": " << head_region);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load head_region");
        n_.shutdown();
    }

	for(int i=0;i<goal_number;i++)
	{
		string sname;
		stringstream ss;
		ss<<"goal_"<<i+1;
		sname=ss.str();
		//sprintf(sname,"goal_%d",i);
		n_.getParam(sname, goal_index[i]);
		ROS_WARN("[param] goal_index: %d", goal_index[i]);   //区域编号输入
	}

	goal_index_out[0] = goal_index[0];//起点
	goal_index_out[1] = head_region;//朝向

	goal_count = 0;
	goal_pub = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    marker_pub = n_.advertise<visualization_msgs::Marker>("car_goal", 1);
	goal_region_pub = n_.advertise<std_msgs::Int8>("/goal_region", 1);
	final_goal_pub = n_.advertise<std_msgs::Int8>("/final_region", 1);

    L1_controller_v2_sub = n_.subscribe("/art_car_controller/goal_finish", 1, &Goal_Publisher::on_goal, this);

	timer1 = n_.createTimer(ros::Duration(1/5), &Goal_Publisher::on_time, this); //10HZ
	initMarker();	
    //registerPub(n_);
	calculate_order_new();	//输出为goal_index_out
	

}
void Goal_Publisher::on_goal(const std_msgs::Int8& goal_num)
{	
	// ROS_WARN("on goal feedback %d",goal_num.data);
	if(ros::ok())//only once :start point
	{
		if(goal_num.data == goal_index_out[goal_count] && goal_count < goal_number)
		{
			goal_count++;	
			
			if( goal_count < goal_number)
			{
				odom_goal.header.frame_id = "map";
				// odom_goal.header.stamp = ros::Time::now();		
				odom_goal.pose.position.x = goal_array[goal_index_out[goal_count]-1][0];
				odom_goal.pose.position.y = goal_array[goal_index_out[goal_count]-1][1];
				odom_goal.pose.orientation.w = 1;
				goal_pub.publish(odom_goal);
			
				std_msgs::Int8 goal_region;
				goal_region.data = goal_index_out[goal_count];
				goal_region_pub.publish(goal_region);
				//cout<<"from on goal"<<goal_count<<endl;

			}
			
			
			//ROS_WARN("goal_region_pub %d",goal_region);
			if(goal_count == goal_number - 1) //告知循迹节点，正在发布的是最后一个节点，以便减小与目标点的距离阈值
			{
				std_msgs::Int8 tmp;
				tmp.data = 0;
				final_goal_pub.publish(tmp);
				ROS_WARN("Publishing Final Goal---~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");   //区域编号输入
			}		
		}
	
	
		odom_goal.header.frame_id = "map";
		// odom_goal.header.stamp = ros::Time::now();		
		odom_goal.pose.position.x = goal_array[goal_index_out[goal_count]-1][0];
		odom_goal.pose.position.y = goal_array[goal_index_out[goal_count]-1][1];
		odom_goal.pose.orientation.w = 1;
		goal_pub.publish(odom_goal);
		
		ROS_WARN("goal_number=%d,goal_count=%d", goal_number,goal_count);
	}
}
void Goal_Publisher::on_time(const ros::TimerEvent&)
{	
//	 ROS_WARN("goal_count=%d,goal_number=%d",goal_count,goal_number);
	if(ros::ok())//only once :start point
	{
		//querry and publish
		if(!goal_count)
		{
			odom_goal.header.frame_id = "map";
			// odom_goal.header.stamp = ros::Time::now();		
			odom_goal.pose.position.x = goal_array[goal_index_out[0]-1][0];
			odom_goal.pose.position.y = goal_array[goal_index_out[0]-1][1];
			odom_goal.pose.orientation.w = 1;
			goal_pub.publish(odom_goal);
			
			std_msgs::Int8 goal_region;
			goal_region.data = goal_index_out[goal_count];
			goal_region_pub.publish(goal_region);
		}

	
		sphere.pose.position.x = goal_array[goal_index_out[goal_count]-1][0];
		sphere.pose.position.y = goal_array[goal_index_out[goal_count]-1][1];
		marker_pub.publish(sphere);
		if(goal_count == goal_number)
		{
			ROS_WARN("|||=============================================||||");
			ROS_WARN("|||                                             ||||");				
			ROS_WARN("|||=====@ Navi Finished - Congratulations! @====||||");
			ROS_WARN("|||                                             ||||");						
			ROS_WARN("|||=============================================||||");	
			ros::shutdown();		
			return;
		}
		//ROS_WARN("publish goal%d,region %d,%d left", goal_count+1,goal_index[goal_count],goal_number-goal_count);
	}
}

/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "Goal_Publisher");
	Goal_Publisher goal_Publisher;
	while(ros::ok())
    	ros::spin();
    return 0;
}





