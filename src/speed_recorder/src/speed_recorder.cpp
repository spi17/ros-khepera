#include "ros/ros.h"
#include <arpa/inet.h>
#include <ros/network.h>
#include <geometry_msgs/Twist.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fstream>

void chatterCallback(const geometry_msgs::Twist cmd_msg){
	
	std::ofstream myfile;
	myfile.open ("/home/santi/data/current_speed.txt", std::fstream::in | std::fstream::out | std::fstream::app);
	myfile << "Linear: ";
	myfile << cmd_msg.linear.x;
	myfile << " ";
	myfile << "Angular: ";
	myfile << cmd_msg.angular.z;
	myfile << " ";
	myfile << "Timestamp: ";
	myfile << ros::Time::now();
	myfile << " \n";
	myfile.close();

}

int main(int argc, char **argv)
{   
	ros::init(argc, argv, "speed_recorder");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("speed", 100, chatterCallback);
	
	ros::spin();

  return 0;
}

