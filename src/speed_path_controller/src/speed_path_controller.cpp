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

void saveCommandInFile(geometry_msgs::Twist cmd_msg){

  std::ofstream myfile;
	myfile.open ("/home/santi/data/command_speed.txt", std::fstream::in | std::fstream::out | std::fstream::app);
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
	ros::init(argc, argv, "speed_path_controller");
  if (argc != 2){
		ROS_ERROR("Need type of path as argument.\n ");
		return -1;
	}

  int PATH = atoi(argv[1]);

	ros::NodeHandle nh_;

	ros::Publisher twist_pub_;
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  geometry_msgs::Twist cmd_msg;

  while(ros::ok()){

    if(PATH == 1) {
        cmd_msg.linear.x = 0.1;
        cmd_msg.angular.z = 0.0;
        twist_pub_.publish(cmd_msg);
        saveCommandInFile(cmd_msg);
        ros::Duration(5).sleep();
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.3927;
        twist_pub_.publish(cmd_msg);
        saveCommandInFile(cmd_msg);
        ros::Duration(4).sleep();

        cmd_msg.linear.x = 0.1;
        cmd_msg.angular.z = 0.0;
        twist_pub_.publish(cmd_msg);
        saveCommandInFile(cmd_msg);
        ros::Duration(5).sleep();
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.3927;
        twist_pub_.publish(cmd_msg);
        saveCommandInFile(cmd_msg);
        ros::Duration(4).sleep();

        cmd_msg.linear.x = 0.1;
        cmd_msg.angular.z = 0.0;
        twist_pub_.publish(cmd_msg);
        saveCommandInFile(cmd_msg);
        ros::Duration(5).sleep();
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.3927;
        twist_pub_.publish(cmd_msg);
        saveCommandInFile(cmd_msg);
        ros::Duration(4).sleep();

        cmd_msg.linear.x = 0.1;
        cmd_msg.angular.z = 0.0;
        twist_pub_.publish(cmd_msg);
        saveCommandInFile(cmd_msg);
        ros::Duration(5).sleep();
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.3927;
        twist_pub_.publish(cmd_msg);
        saveCommandInFile(cmd_msg);
        ros::Duration(4).sleep();

        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;
        twist_pub_.publish(cmd_msg);
        saveCommandInFile(cmd_msg);
        ros::Duration(10).sleep();        

    } else if (PATH == 2){
        cmd_msg.linear.x = 0.2;
        cmd_msg.angular.z = 0.4;
        twist_pub_.publish(cmd_msg);
        saveCommandInFile(cmd_msg);
        ros::Duration(15.708).sleep();

        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;
        twist_pub_.publish(cmd_msg);
        saveCommandInFile(cmd_msg);
        ros::Duration(10).sleep();     

    } else {
      printf("Other paths...\n");
    }

    ros::spinOnce();
  }

  return 0;
}