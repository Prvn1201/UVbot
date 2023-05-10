// 10/3/2022 07::50

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/UInt16MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstdio>  // for EOF
#include <string>
#include <sstream>
#include <vector>
#include <bits/stdc++.h>

#include <moverobotic_simple_navigation/moverobotic_simple_navigationConfig.h>

typedef moverobotic_simple_navigation::moverobotic_simple_navigationConfig dynamicConfig;

class MOVEROBOTIC_SIMPLE_NAVIGATION
{
	public:
		MOVEROBOTIC_SIMPLE_NAVIGATION();
		~MOVEROBOTIC_SIMPLE_NAVIGATION();

	private:
		void DynamicCB(dynamicConfig &config, uint32_t level);
		void ProcessCB(const ros::TimerEvent& e);
		void HomeCB(const std_msgs::UInt16MultiArray::ConstPtr& msg);
		bool parseCoordinate(std::string input, geometry_msgs::PoseWithCovarianceStamped &output);

		ros::NodeHandle node_;
		ros::NodeHandle node_pri;

		ros::Subscriber home_subscriber;
		ros::Publisher initial_pose_publisher;

		dynamic_reconfigure::Server<dynamicConfig> dynamicServer;
		dynamic_reconfigure::Server<dynamicConfig>::CallbackType dynamicServerType;
		boost::recursive_mutex config_mutex;
		dynamicConfig config;

		bool homeInit;
		bool homePin;
		int homeState;
		ros::Time homeTime;
		geometry_msgs::PoseWithCovarianceStamped msg_home;
		tf2_ros::Buffer tfBuffer;
		tf2_ros::TransformListener tfListener;
};

MOVEROBOTIC_SIMPLE_NAVIGATION::MOVEROBOTIC_SIMPLE_NAVIGATION() : node_(""), node_pri("~"), tfListener(tfBuffer)
{
	dynamicServerType = boost::bind(&MOVEROBOTIC_SIMPLE_NAVIGATION::DynamicCB, this, _1, _2);
	dynamicServer.setCallback(dynamicServerType);
	dynamicServer.getConfigDefault(config);

	node_pri.param<std::string>("base_frame_id", config.base_frame_id, "base_link");
	node_pri.param<std::string>("map_frame_id", config.map_frame_id, "map");
	node_pri.param<std::string>("home_coordinate", config.home_coordinate, "0 0 0 0 0 0 1");
	node_pri.param<int>("home_button", config.home_button, 0);

	home_subscriber = node_.subscribe<std_msgs::UInt16MultiArray> ("gpi_read", 1, boost::bind(&MOVEROBOTIC_SIMPLE_NAVIGATION::HomeCB,this, _1));
	initial_pose_publisher = node_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("initialpose", 1, false);

	homeInit = true;
	homePin = true;
	homeState = 0;
	geometry_msgs::PoseWithCovarianceStamped temp_msg;
	if(parseCoordinate(config.home_coordinate, temp_msg))
	{
		msg_home = temp_msg;
		config.home_coordinate = std::to_string(msg_home.pose.pose.position.x) + " ";
		config.home_coordinate += std::to_string(msg_home.pose.pose.position.y) + " ";
		config.home_coordinate += std::to_string(msg_home.pose.pose.position.z) + " ";
		config.home_coordinate += std::to_string(msg_home.pose.pose.orientation.x) + " ";
		config.home_coordinate += std::to_string(msg_home.pose.pose.orientation.y) + " ";
		config.home_coordinate += std::to_string(msg_home.pose.pose.orientation.z) + " ";
		config.home_coordinate += std::to_string(msg_home.pose.pose.orientation.w);
		dynamicServer.updateConfig(config);
	}
	else
	{
		config.home_coordinate = "0 0 0 0 0 0 1";
		dynamicServer.updateConfig(config);
		msg_home.header.stamp = ros::Time::now();
		msg_home.header.frame_id = config.map_frame_id;
		msg_home.pose.pose.position.x = 0;
		msg_home.pose.pose.position.y = 0;
		msg_home.pose.pose.position.z = 0;
		msg_home.pose.pose.orientation.x = 0;
		msg_home.pose.pose.orientation.y = 0;
		msg_home.pose.pose.orientation.z = 0;
		msg_home.pose.pose.orientation.w = 1;
	}
}

MOVEROBOTIC_SIMPLE_NAVIGATION::~MOVEROBOTIC_SIMPLE_NAVIGATION()
{
}

void MOVEROBOTIC_SIMPLE_NAVIGATION::DynamicCB(dynamicConfig &config_new, uint32_t level)
{
	config.base_frame_id = config_new.base_frame_id;
	config.map_frame_id = config_new.map_frame_id;

	geometry_msgs::PoseWithCovarianceStamped temp_msg;
	if(parseCoordinate(config_new.home_coordinate, temp_msg))
	{
		msg_home = temp_msg;
		config.home_coordinate = std::to_string(msg_home.pose.pose.position.x) + " ";
		config.home_coordinate += std::to_string(msg_home.pose.pose.position.y) + " ";
		config.home_coordinate += std::to_string(msg_home.pose.pose.position.z) + " ";
		config.home_coordinate += std::to_string(msg_home.pose.pose.orientation.x) + " ";
		config.home_coordinate += std::to_string(msg_home.pose.pose.orientation.y) + " ";
		config.home_coordinate += std::to_string(msg_home.pose.pose.orientation.z) + " ";
		config.home_coordinate += std::to_string(msg_home.pose.pose.orientation.w);
		config_new.home_coordinate = config.home_coordinate;
	}
	else
	{
		config_new.home_coordinate = config.home_coordinate;
	}
	
	config.home_button = config_new.home_button;

	if(config_new.home_get)
	{
		config_new.home_get = false;
		geometry_msgs::TransformStamped transformStamped;
		try
		{
			transformStamped = tfBuffer.lookupTransform(config.map_frame_id, config.base_frame_id, ros::Time(0));
			msg_home.pose.pose.position.x = transformStamped.transform.translation.x;
			msg_home.pose.pose.position.y = transformStamped.transform.translation.y;
			msg_home.pose.pose.position.z = transformStamped.transform.translation.z;
			msg_home.pose.pose.orientation = transformStamped.transform.rotation;
			config.home_coordinate = std::to_string(msg_home.pose.pose.position.x) + " ";
			config.home_coordinate += std::to_string(msg_home.pose.pose.position.y) + " ";
			config.home_coordinate += std::to_string(msg_home.pose.pose.position.z) + " ";
			config.home_coordinate += std::to_string(msg_home.pose.pose.orientation.x) + " ";
			config.home_coordinate += std::to_string(msg_home.pose.pose.orientation.y) + " ";
			config.home_coordinate += std::to_string(msg_home.pose.pose.orientation.z) + " ";
			config.home_coordinate += std::to_string(msg_home.pose.pose.orientation.w);
			config_new.home_coordinate = config.home_coordinate;
		}
		catch (tf2::TransformException &ex)
		{
			ROS_WARN("%s",ex.what());
		}
	}

	if(config_new.home_set)
	{
		config_new.home_set = false;
		initial_pose_publisher.publish(msg_home);
		ROS_WARN_STREAM("RESET TO HOME LOCALIZATION");
	}
}

void MOVEROBOTIC_SIMPLE_NAVIGATION::HomeCB(const std_msgs::UInt16MultiArray::ConstPtr& msg)
{
	bool homeTemp = (bool)msg->data[config.home_button];
	if(homeInit)
	{
		homeInit = false;
	}
	else
	{
		if(homePin>homeTemp)
		{
			homeTime = ros::Time::now();
		}
		else if(homePin<homeTemp)
		{
			if((ros::Time::now() - homeTime).toSec() >= 1.0)
			{
				initial_pose_publisher.publish(msg_home);
				ROS_WARN_STREAM("RESET TO HOME LOCALIZATION");
			}
		}
		else if(homePin==homeTemp && !homeTemp)
		{
		}
		else if(homePin==homeTemp && homeTemp)
		{
			homeTime = ros::Time::now();
		}
	}
	homePin = homeTemp;
}

bool MOVEROBOTIC_SIMPLE_NAVIGATION::parseCoordinate(std::string input, geometry_msgs::PoseWithCovarianceStamped &output)
{
	std::vector<double> temp_vector;
	std::stringstream check1(input);
	std::string intermediate;
	while(std::getline(check1, intermediate, ' '))
	{
		temp_vector.push_back(std::stod(intermediate));
	}
	if(temp_vector.size()==6)
	{
		output.header.stamp = ros::Time::now();
		output.header.frame_id = config.map_frame_id;
		output.pose.pose.position.x = temp_vector[0];
		output.pose.pose.position.y = temp_vector[1];
		output.pose.pose.position.z = temp_vector[2];
		tf2::Quaternion temp_quaternion;
		temp_quaternion.setRPY(temp_vector[3], temp_vector[4], temp_vector[5]);
		temp_quaternion.normalize();
		tf2::convert(temp_quaternion, output.pose.pose.orientation);
		return true;
	}
	else if(temp_vector.size()==7)
	{
		output.header.stamp = ros::Time::now();
		output.header.frame_id = config.map_frame_id;
		output.pose.pose.position.x = temp_vector[0];
		output.pose.pose.position.y = temp_vector[1];
		output.pose.pose.position.z = temp_vector[2];
		output.pose.pose.orientation.x = temp_vector[3];
		output.pose.pose.orientation.y = temp_vector[4];
		output.pose.pose.orientation.z = temp_vector[5];
		output.pose.pose.orientation.w = temp_vector[6];
		tf2::Quaternion temp_quaternion;
		tf2::convert(output.pose.pose.orientation, temp_quaternion);
		temp_quaternion.normalize();
		tf2::convert(temp_quaternion, output.pose.pose.orientation);
		return true;
	}
	else
	{
		return false;
	}
}

void MOVEROBOTIC_SIMPLE_NAVIGATION::ProcessCB(const ros::TimerEvent& e)
{
	boost::recursive_mutex::scoped_lock lock(config_mutex);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "moverobotic_simple_navigation");

	MOVEROBOTIC_SIMPLE_NAVIGATION _MOVEROBOTIC_SIMPLE_NAVIGATION;
	ros::AsyncSpinner spinner(0);
	spinner.start();
	ros::waitForShutdown();
	ros::shutdown();

	return 0;
}