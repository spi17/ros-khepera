#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
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
const float wheelDistance = 0.088;

const float arg1 = 0.0105;
const float arg2 = 0.23753;

int main(int argc, char** argv){
	
	ros::init(argc, argv, "speed_sensor");
	if (argc != 3){
		ROS_ERROR("Need server port as argument (first) and khepera number (second) as arguments.\n ");
		return -1;
	}
	
	int PORT = atoi(argv[1]);
	khepera_name = argv[2];
	printf("PORT: %d\n", PORT);

	ros::Publisher current_speed_pub;
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
	current_speed_pub = node.advertise<geometry_msgs::Twist>("speed", 50);

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

		//printf("/***************************************************/\n");
		//printf("\n");
		
		//printf("Mensaje recibido: %s\n",recv_buffer);
		//decodes it
		while(recv_buffer[i]){
			if(recv_buffer[i]=='C' && recv_buffer[i+1]=='S'){//CS: code for current speed
				i=i+3; //Jump after L
					while(recv_buffer[i]!='R' && recv_buffer[i]){
						if ((recv_buffer[i]>='0' && recv_buffer[i]<='9')||recv_buffer[i]=='.'||recv_buffer[i]=='-'){
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
						if ((recv_buffer[i]>='0' && recv_buffer[i]<='9')||recv_buffer[i]=='.'||recv_buffer[i]=='-'){
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
		//converts string to integer and translates the number to meters
		speed_left= -1 * atof(str_speed_left);
		speed_right=atof(str_speed_right);
		//printf("En formato entero: CS L:%f R:%f\n", speed_left, speed_right);

		// convert from angular of wheels to linear and angular of robot

		geometry_msgs::Twist currentSpeed;

		currentSpeed.linear.x = arg1 * (speed_right + speed_left);

		currentSpeed.angular.z = arg2 * (speed_right - speed_left);

		// publish to topic

		current_speed_pub.publish(currentSpeed);

		ros::spinOnce();
	}
	return 0;
};
