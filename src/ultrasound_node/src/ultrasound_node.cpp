#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
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

#define BUFLEN 64
#define PORT 3002 //change in function of the khepera number.


int main(int argc, char** argv){

	ros::Publisher scan_pub;
	ros::Time last_time;

	//Here is where ultrasound measures are going to be placed
	char str_ultrasound_left[5], str_ultrasound_front_left[5], str_ultrasound_front[5];
	char str_ultrasound_right[5], str_ultrasound_front_right[5];
	
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
	
	//ROS 
	ros::init(argc, argv, "ultrasound_node");
	ros::NodeHandle node;
	scan_pub = node.advertise<sensor_msgs::LaserScan>("/scan_US", 50);

	last_time=ros::Time::now();

	while (ros::ok()){

		char recv_buffer[BUFLEN]={0};
		int recv_len;
		int i=0, j=0;
		float ultrasound_left, ultrasound_front_left, ultrasound_front;
		float ultrasound_right, ultrasound_front_right;
		
		//receives message
		if (recvfrom(sock, recv_buffer, BUFLEN, 0, (struct sockaddr *)&sender, &sender_len)==-1){
			printf("Could not receive connection: Error %d\n", errno);
			exit(1);
		}
		
		//decodes it
		while(recv_buffer[i]){
			//j=0;
			if(recv_buffer[i]=='U'){//U: code for ultrasound
				i=i+2;
				while(recv_buffer[i++]!='L'); //I get to the number of L sensor
				j=0;
					while(recv_buffer[i]!='F' && recv_buffer[i]){
						if (recv_buffer[i]>='0' && recv_buffer[i]<='9'){
							str_ultrasound_left[j]=recv_buffer[i]; //I copy L sensor
							j++;
							i++;
						}
						else{
							i++;
						}
					}
					str_ultrasound_left[j]=0;
					j=0;
					while(recv_buffer[i++]!='F'); //I get to the number of FL sensor

					while(recv_buffer[i]!='F' && recv_buffer[i]){
						if (recv_buffer[i]>='0' && recv_buffer[i]<='9'){
							str_ultrasound_front_left[j]=recv_buffer[i]; //I copy FL sensor
							j++;
							i++;
						}
						else{
							i++;
						}
					}
					str_ultrasound_front_left[j]=0;
					j=0;
					while(recv_buffer[i++]!='F'); //I get to the number of F sensor

					while(recv_buffer[i]!='F' && recv_buffer[i]){
						if (recv_buffer[i]>='0' && recv_buffer[i]<='9'){
							str_ultrasound_front[j]=recv_buffer[i]; //I copy F sensor
							j++;
							i++;
						}
						else{
							i++;
						}
					}
					str_ultrasound_front[j]=0;
					j=0;
					while(recv_buffer[i++]!='F'); //I get to the number of FR sensor
					i++;

					while(recv_buffer[i]!='R' && recv_buffer[i]){
						//printf("%d\n",i);
						if (recv_buffer[i]>='0' && recv_buffer[i]<='9'){
							str_ultrasound_front_right[j]=recv_buffer[i]; //I copy FR sensor
							j++;
							i++;
						}
						else{
							i++;
						}
					}
					str_ultrasound_front_right[j]=0;
					j=0;
					while(recv_buffer[i++]!='R'); //I get to the number of R sensor

					while(recv_buffer[i]){
						if (recv_buffer[i]>='0' && recv_buffer[i]<='9'){
							str_ultrasound_right[j]=recv_buffer[i];
							j++;
							i++;
						}
						else{
							i++;
						}
					}
					str_ultrasound_right[j]=0;


				}
			i++;
			}

		fflush(stdout);
		
		//converts string to integer and translates the number to meters
		ultrasound_left=atoi(str_ultrasound_left);
		ultrasound_front_left=atoi(str_ultrasound_front_left);
		ultrasound_front=atoi(str_ultrasound_front);
		ultrasound_right=atoi(str_ultrasound_right);
		ultrasound_front_right=atoi(str_ultrasound_front_right);

		sensor_msgs::LaserScan scan;
	    scan.header.stamp = ros::Time::now();
	    scan.header.frame_id = "/base_laser";


	    scan.angle_min = -1.570796; //// angle_min is measured in x
	    scan.angle_max = 1.570796;
	    scan.angle_increment = 3.14 / 6;

	    scan.time_increment = 0.0075;
	    scan.scan_time=(ros::Time::now()-last_time).toSec();
	    last_time=ros::Time::now();

	    scan.range_min = 0.30;
	    scan.range_max = 1.5;

	    scan.ranges.resize(5);//six is the number of sensors.

	    //Now, the IR measures:

	    scan.ranges[0]= ultrasound_left/1000.0; //left sensor info (in meters)
	    scan.ranges[1]= ultrasound_front_left/1000.0; //front left sensor info (in meters)
	    scan.ranges[2]= ultrasound_front/1000.0; //front sensor info (in meters), etc
	    scan.ranges[3]= ultrasound_front_right/1000.0;
	    scan.ranges[4]= ultrasound_right/1000.0;

		scan_pub.publish(scan);

		ros::spinOnce();
	}
	return 0;
};
