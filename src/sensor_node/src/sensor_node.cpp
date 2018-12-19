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
#include <iostream>
#include <string.h>


#define BUFLEN 64

std::string khepera_name;

	//Here is where infrared measures are going to be placed
char str_infrared_left[5], str_infrared_front_side_left[5], str_infrared_front_left[5];
char str_infrared_right[5], str_infrared_front_side_right[5], str_infrared_front_right[5];

float sensorToMRightSensor (int sensor){
	//function that uses interpolation to return the m to the obstacle
	float meters = 0, offset= 0.07;
	if (sensor > 3900) meters = 0.0;
	else if (sensor > 800) meters=  - (sensor-3900.0)/77500.0;
	else if (sensor > 380 && sensor <= 800) meters = 0.06-(sensor-380.0)/21000.0;
	else return 1;
	printf("\n");
	std::cout << khepera_name << ": ";
	ROS_INFO("Right sensor: meters: %f, sensor: %d", meters, sensor);
	return meters+offset;
}

float sensorToMFrontSideRightSensor (int sensor){
	//function that uses interpolation to return the m to the obstacle
	float meters = 0, offset= 0.07;
	if (sensor > 3900) meters = 0.0;
	else if (sensor > 730) meters= - (sensor-3900.0)/79250.0;
	else if (sensor > 250 && sensor <= 730) meters = 0.08-(sensor-250.0)/13750.0;
	else return 1;
	printf("\n");
	std::cout << khepera_name << ": ";
	ROS_INFO("FrontSideRight sensor: meters: %f, sensor: %d", meters, sensor);
	return meters+offset;
}

float sensorToMFrontRightSensor (int sensor){
	//function that uses interpolation to return the m to the obstacle
	float meters = 0, offset= 0.07;
	if (sensor > 3900) meters = 0.0;
	else if (sensor > 1100) meters= 0.04 - (sensor-1100.0)/70000.0;
	else if (sensor > 330 && sensor <= 1100) meters = 0.08-(sensor-330.0)/19250.0;
	else return 1;
	printf("\n");
	std::cout << khepera_name << ": ";
	ROS_INFO("FrontRight sensor: meters: %f, sensor: %d", meters, sensor);
	return meters+offset;
}


float sensorToMFrontLeftSensor (int sensor){
	//function that uses interpolation to return the m to the obstacle
	float meters = 0, offset= 0.07;
	if (sensor > 3900) meters = 0.0;
	else if (sensor > 520) meters= - (sensor-3900.0)/84500.0;
	else if (sensor > 250 && sensor <= 520) meters = 0.08-(sensor-250.0)/8250.0;
	else return 1;
	printf("\n");
	std::cout << khepera_name << ": ";
	ROS_INFO("FrontLeft sensor: meters: %f, sensor: %d", meters, sensor);
	return meters+offset;
}

float sensorToMFrontSideLeftSensor (int sensor){
	//function that uses interpolation to return the m to the obstacle
	float meters = 0, offset= 0.07;
	if (sensor > 3900) meters = 0.0;
	else if (sensor > 890) meters= - (sensor-3900.0)/75250.0;
	else if (sensor > 470 && sensor <= 890) meters = 0.06-(sensor-470.0)/21000.0;
	else return 1;
	printf("\n");
	std::cout << khepera_name << ": ";
	ROS_INFO("FrontSideLeft sensor: meters: %f, sensor: %d", meters, sensor);
	return meters+offset;
}

float sensorToMLeftSensor (int sensor){
	//function that uses interpolation to return the m to the obstacle
	float meters = 0, offset= 0.07;
	if (sensor > 3900) meters = 0.0;
	else if (sensor > 1100) meters= 0.04 - (sensor-1100.0)/70000.0;
	else if (sensor > 350 && sensor <= 1100) meters = 0.04-(sensor-1100.0)/20000.0;
	else return 1;
	printf("\n");
	std::cout << khepera_name << ": ";
	ROS_INFO("Left sensor: meters: %f, sensor: %d", meters, sensor);
	return meters+offset;
}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "sensor_node");
	if (argc != 3){
		ROS_ERROR("Need server port as argument (first) and khepera number (second) as arguments.\n ");
		return -1;
	}
	
	int PORT = atoi(argv[1]);
	khepera_name = argv[2];

	printf("PORT: %d \n", PORT);

	ros::Publisher scan_pub;
	ros::Time last_time;

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
	scan_pub = node.advertise<sensor_msgs::LaserScan>("scan", 50);

	last_time=ros::Time::now();

	while (ros::ok()){

		char recv_buffer[BUFLEN]={0};
		int recv_len;
		int i=0, j=0;
		float infrared_left, infrared_front_side_left, infrared_front_left;
		float infrared_right, infrared_front_side_right, infrared_front_right;
		
		//receives message
		if (recvfrom(sock, recv_buffer, BUFLEN, 0, (struct sockaddr *)&sender, &sender_len)==-1){
			printf("Could not receive connection: Error %d\n", errno);
			exit(1);
		}
		
		//decodes it
		while(recv_buffer[i]){
			//j=0;
			if(recv_buffer[i]=='I'){//I: code for infrared
				i=i+2;
				while(recv_buffer[i++]!='L');
				j=0;
					while(recv_buffer[i]!='F' && recv_buffer[i]){
						if (recv_buffer[i]>='0' && recv_buffer[i]<='9'){
							str_infrared_left[j]=recv_buffer[i];
							j++;
							i++;
						}
						else{
							i++;
						}
					}
					str_infrared_left[j]=0;
					j=0;
					while(recv_buffer[i++]!='F');

					while(recv_buffer[i]!='F' && recv_buffer[i]){
						if (recv_buffer[i]>='0' && recv_buffer[i]<='9'){
							str_infrared_front_side_left[j]=recv_buffer[i];
							j++;
							i++;
						}
						else{
							i++;
						}
					}
					str_infrared_front_side_left[j]=0;
					j=0;
					while(recv_buffer[i++]!='F');

					while(recv_buffer[i]!='F' && recv_buffer[i]){
						if (recv_buffer[i]>='0' && recv_buffer[i]<='9'){
							str_infrared_front_left[j]=recv_buffer[i];
							j++;
							i++;
						}
						else{
							i++;
						}
					}
					str_infrared_front_left[j]=0;
					j=0;
					while(recv_buffer[i++]!='F');

					while(recv_buffer[i]!='F' && recv_buffer[i]){
						//printf("%d\n",i);
						if (recv_buffer[i]>='0' && recv_buffer[i]<='9'){
							str_infrared_front_right[j]=recv_buffer[i];
							j++;
							i++;
						}
						else{
							i++;
						}
					}
					str_infrared_front_right[j]=0;
					j=0;
					while(recv_buffer[i++]!='F');

					while((recv_buffer[i]!='R' || recv_buffer[i-1]!=' ') && recv_buffer[i]){
						//printf("[%d]\n",i);
						if (recv_buffer[i]>='0' && recv_buffer[i]<='9'){
							str_infrared_front_side_right[j]=recv_buffer[i];
							j++;
							i++;
						}
						else{
							i++;
						}
					}
					str_infrared_front_side_right[j]=0;
					j=0;
					while(recv_buffer[i++]!='R');

					while(recv_buffer[i]){
						if (recv_buffer[i]>='0' && recv_buffer[i]<='9'){
							str_infrared_right[j]=recv_buffer[i];
							j++;
							i++;
						}
						else{
							i++;
						}
					}
					str_infrared_right[j]=0;


				}
			i++;
			}

		fflush(stdout);
		
		//converts string to integer and translates the number to meters
		infrared_left=sensorToMLeftSensor(atoi(str_infrared_left));
		infrared_front_side_left=sensorToMFrontSideLeftSensor(atoi(str_infrared_front_side_left));
		infrared_front_left=sensorToMFrontLeftSensor(atoi(str_infrared_front_left));
		infrared_right=sensorToMRightSensor(atoi(str_infrared_right));
		infrared_front_side_right=sensorToMFrontSideRightSensor(atoi(str_infrared_front_side_right));
		infrared_front_right=sensorToMFrontRightSensor(atoi(str_infrared_front_right));
		
		//printf("Recepcion de sensores: %f ,%f ,%f ,%f ,%f ,%f \n", 
		//infrared_left, infrared_front_side_left, infrared_front_left, infrared_right, infrared_front_side_right, infrared_front_right);

		sensor_msgs::LaserScan scan;
	    scan.header.stamp = ros::Time::now();
	    scan.header.frame_id = 
		// "/"+khepera_name+
		"base_laser";


	    scan.angle_min = -1.309; //// angle_min is measured in x
	    scan.angle_max = 1.309;
	    scan.angle_increment = 0.5236;//150/5

	    scan.time_increment = 0.003;
	    scan.scan_time=(ros::Time::now()-last_time).toSec();
	    last_time=ros::Time::now();

	    scan.range_min = 0.065;
	    scan.range_max = 0.215;

	    scan.ranges.resize(6);//six is the number of sensors.

	    //Now, the IR measures:

	    scan.ranges[5]= infrared_left; //left sensor info (in meters)
	    scan.ranges[4]= infrared_front_side_left; //front side left sensor info (in meters)
	    scan.ranges[3]= infrared_front_left; //front left sensor info (in meters), etc
	    scan.ranges[2]= infrared_front_right;
	    scan.ranges[1]= infrared_front_side_right;
	    scan.ranges[0]= infrared_right;

		scan_pub.publish(scan);
		
		ros::spinOnce();
	}
	return 0;
};
