#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
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
#include <iostream>
#include <string.h>


#define BUFLEN 64

std::string khepera_name;

const float pi = 3.14159265359;
static geometry_msgs::Pose previousPose;
static float previousSpeedLeft = 0;
static float previousSpeedRight = 0;
static float theta = 0;
const float wheelDistance = 0.088;
static ros::Time last_time;

geometry_msgs::Pose getPosition(float speed_left, float speed_right){

	geometry_msgs::Pose pose;
	ros::Time this_time = ros::Time::now();

	float secs = (this_time - last_time).toSec();

	float SL = secs*previousSpeedLeft;
	float SR = secs*previousSpeedRight;

	float meanDistance = (SL + SR)/2;
	theta += (SR - SL)/wheelDistance;

	if(theta > 2*pi){
		theta -= 2*pi;
	} else if(theta < -2*pi){
		theta += 2*pi;
	}

	printf("SL: %f , SR: %f, theta: %f\n", SL, SR, theta);
	printf("speedl: %f, speedr: %f\n", speed_left, speed_right);
	printf("prevspeedl: %f, prevspeedr: %f\n", previousSpeedLeft, previousSpeedRight);
	printf("tiempo transcurrido: %f \n", secs);
	printf("Coseno: %f, seno: %f\n", cos(theta), sin(theta));
	pose.position.x = previousPose.position.x + meanDistance*cos(theta);
	pose.position.y = previousPose.position.y + meanDistance*sin(theta);
	pose.position.z = 0;

	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = theta;

	previousPose = pose;

	previousSpeedLeft = speed_left;
	previousSpeedRight = speed_right;

	last_time = this_time;

	return pose;
}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "dead_reckoning");
	if (argc != 3){
		ROS_ERROR("Need server port as argument (fist) and khepera number (second) as arguments.\n ");
		return -1;
	}
	
	int PORT = atoi(argv[1]);
	khepera_name = argv[2];
	printf("PORT: %d\n", PORT);

	ros::Publisher pose_pub;
	//ros::Time last_time;

	//Socket variables
	int sock;
	struct sockaddr_in address;
	struct sockaddr_in sender;
	int res;
	socklen_t sender_len = sizeof(struct sockaddr_in);

	// Initialize the address
	address.sin_family = AF_INET;
	address.sin_port = htons(PORT);
	address.sin_addr.s_addr = htonl(INADDR_ANY);

	// Create socket and bind it
	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock<0) {
		printf("Could not create socket: Error %d\n", errno);
		exit(1);
	}
	
	res=bind(sock, (struct sockaddr *)&address, sizeof(address));
	if (res==-1) { //checked
		printf("Could not bind socket to port %d: Error %d\n", PORT, errno);
		exit(1);
	}
	
	ros::NodeHandle node;
	pose_pub = node.advertise<geometry_msgs::Pose>("pose", 50);

	while (ros::ok()){

		char recv_buffer[BUFLEN]={0};
		int recv_len;
		int i=0, j=0;
		float speed_left;
		float speed_right;
		
		//Here is where speed measures are going to be placed
		char str_speed_left[5] = "";
		char str_speed_right[5] = "";

		//receives message
		if (recvfrom(sock, recv_buffer, BUFLEN, 0, (struct sockaddr *)&sender, &sender_len)==-1){
			printf("Could not receive connection: Error %d\n", errno);
			exit(1);
		}

		printf("/***************************************************/\n");
		printf("\n");
		
		printf("Mensaje recibido: %s\n",recv_buffer);
		//decodes it
		while(recv_buffer[i]){
			if(recv_buffer[i]=='V'){//V: code for speed
				i=i+2;
					while(recv_buffer[i]!='R' && recv_buffer[i]){
						if ((recv_buffer[i]>='0' && recv_buffer[i]<='9')||recv_buffer[i]=='-'){
							str_speed_left[j]=recv_buffer[i];
							j++;
							i++;
						}
						else{
							i++;
						}
					}
					j=0;
					while(recv_buffer[i]){
						if ((recv_buffer[i]>='0' && recv_buffer[i]<='9')||recv_buffer[i]=='-'){
							str_speed_right[j]=recv_buffer[i];
							j++;
							i++;
						}
						else{
							i++;
						}
					}
			}
			i++;
		}
		fflush(stdout);
		//printf("En formato char: L:%s R:%s\n", str_speed_left, str_speed_right);
		//converts string to integer and translates the number to meters
		speed_left=atoi(str_speed_left);
		speed_right=atoi(str_speed_right);
		//printf("En formato entero: L:%f R:%f\n", speed_left, speed_right);

		geometry_msgs::Pose pose;
		pose = getPosition(speed_left/150000,speed_right/150000);
		printf("PosX:%f PosY:%f PosZ:%f OriX:%f OriY:%f OriZ:%f OriW:%f\n", pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
		
		printf("/***************************************************/\n");
		printf("\n");
		
		pose_pub.publish(pose);
		
		ros::spinOnce();
	}
	return 0;
};
