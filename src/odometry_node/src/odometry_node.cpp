//This node receives a pose message from mocap_optitrack, calculates
//speed by difference quotients, and publishes three messages, two in
//the tf with the info of the transformations between /odom (world)
//and /base_link (base of the robot), corrected because opitrack gives
//twice the distance between points); and the tranformations between the
//base_link and the base_laser.

//The other message that it publishes is the odometry message in the
//odom topic.

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <arpa/inet.h>
#include <signal.h>
#include <string.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <math.h>


ros::Publisher odom_pub;

std::string khepera_name;

/*These functions calculate the speed in X, in Y and in Theta
 * They take the last position and the one before, they substract 
 * the latter to the former and then they divide it by the seconds
 *  that have passed since the last call to the function. 
 * Therefore, they return the speed in mm/s or rads/s*/
 
float SpeedX(float position, float last_position){
	float speed;
	ros::Time current_time = ros::Time::now();
	static ros::Time last_time = ros::Time::now(); 
	float time_increment = (current_time - last_time).toSec();
	static bool inicio = 1;	

	if (inicio == 1){
		inicio = 0;
		return 0;		
	}
	speed = time_increment > 0.001 ? (position-last_position)/time_increment:0;
	last_time=current_time;
	printf("Speed X: %f\n", speed);
	return speed;
}
float SpeedY(float position, float last_position){
	float speed;
	ros::Time current_time = ros::Time::now();
	static ros::Time last_time = ros::Time::now(); 
	float time_increment = (current_time - last_time).toSec();
	static bool inicio = 1;	

	if (inicio == 1){
		inicio = 0;
		return 0;		
	}
	speed = time_increment > 0.001 ? (position-last_position)/time_increment:0;
	last_time=current_time;
	printf("Speed Y: %f\n", speed);
	return speed;
}
float SpeedTh(float position, float last_position){
	float speed;
	ros::Time current_time = ros::Time::now();
	static ros::Time last_time = ros::Time::now(); 
	float time_increment = (current_time - last_time).toSec();
	static bool inicio = 1;	

	if (inicio == 1){
		inicio = 0;
		return 0;		
	}
	speed = time_increment > 0.001 ? (position-last_position)/time_increment:0;
	last_time=current_time;
	printf("Speed Theta: %f\n", speed);
	return speed;
}


void poseCallback(const geometry_msgs::PoseConstPtr& msg){
	static tf::TransformBroadcaster br, broadcaster;
	static nav_msgs::Odometry odom_ant;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(msg->position.x/2, msg->position.y/2, msg->position.z/2) );

	transform.setRotation( tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w)); 	
	
	//We'll publish tf transformations
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
	broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.0, -0.06)), 
		ros::Time::now(), "base_link", "base_laser"));
	//Both /odom to /base_link and /base_link to /base_laser transforms
	//transform, Timestamp, name of the parent frame, name of the child frame
	
	////Now we have to use the info from optitrack to create the odometry message
	
	geometry_msgs::Quaternion odom_quat;
	odom_quat.x= msg->orientation.x;
	odom_quat.y= msg->orientation.y;
	odom_quat.z= msg->orientation.z;
	odom_quat.w= msg->orientation.w;

	
    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id ="odom"; 
   
    //set the position
    odom.pose.pose.position.x = msg->position.x/2;
	odom.pose.pose.position.y = msg->position.y/2;
    odom.pose.pose.position.z = msg->position.z/2;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
	odom.child_frame_id = "base_link";    
    odom.twist.twist.linear.x = SpeedX(odom.pose.pose.position.x, odom_ant.pose.pose.position.x);
    odom.twist.twist.linear.y = SpeedY(odom.pose.pose.position.y, odom_ant.pose.pose.position.y);
    odom.twist.twist.angular.z = SpeedTh(tf::getYaw(odom.pose.pose.orientation), 
		tf::getYaw(odom_ant.pose.pose.orientation));

    //publish the message
    odom_pub.publish(odom);

	odom_ant=odom;
	
	//we wait 90ms in order to avoid the noise between measurements. 
	usleep(1000*90);
	
}


int main(int argc, char** argv){
	
	ros::init(argc, argv, "odometry_node");

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("pose", 1, &poseCallback); 
	//only one in the queue because we are going to check this topic once in 90ms to avoid the
	//noise between measurements. 
	odom_pub = node.advertise<nav_msgs::Odometry>("odom", 50);

	ros::spin();
	return 0;
};
