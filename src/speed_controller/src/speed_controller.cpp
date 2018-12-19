//base controller
//gets a Twist message and delivers it to the khepera 
//(sends left and right speed of the wheels in this format: L0.5 R0.5)
//It's the client in the socket

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

#define BUFLEN 32
#define NPACK 10
#define PORT 3090
#define WHEELS_DISTANCE 0.08841 //wheels distance
#define WHEELS_RADIUS 0.021 //the wheel radius
    
#define SRV_IP "192.168.1.2" //Put the IP of the Khepera you wish to connect

struct sockaddr_in si_other;//server adress
int s; //socket id
float arg1 = 1000.0 / 21.0;
float arg2 = 50000.0 / 23753.0;

void chatterCallback(const geometry_msgs::Twist cmd_msg){
	
	static int i=0;
	float wl=0, wr=0;
	char buf[BUFLEN];
	
	// convert linear and angular of robot to angular of wheels

	//  |wr|     |1000/21  50000/23753 |   |vl|
	//  |  |  =  |                     | * |  |
	//  |wl|     |1000/21  -50000/23753|   |w |

	wr = (arg1*cmd_msg.linear.x) + (arg2*cmd_msg.angular.z);
	wl = (arg1*cmd_msg.linear.x) - (arg2*cmd_msg.angular.z);

	printf("Speed command: L: %f, R: %f\n", wl, wr);

	fflush(stdin);
	snprintf(buf, BUFLEN, "L%f R%f", wl, wr); //puts the string in buf 
	if (sendto(s, buf, BUFLEN, 0, (const struct sockaddr*) &si_other, sizeof(si_other))==-1){
		//exit(1);
	}
			
	fflush(stdout);	
	
}

int main(int argc, char **argv)
{   
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
		exit(1);
    
	memset((char *) &si_other, 0, sizeof(si_other)); //first, we fill the struct with 0s
	//we put the address of the server; the client adress is not needed.
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(PORT);
	if (inet_aton(SRV_IP, &si_other.sin_addr)==0) {
		//inet.aton transforms an IP direction (SRV_IP) in this format : "127.0.0.1" to binary format,
		//and introduces it in the structure sin_addr
		fprintf(stderr, "inet_aton() failed\n");
		exit(1);
	}
    
	ros::init(argc, argv, "speed_controller");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("cmd_vel", 100, chatterCallback);
	
	ros::spin();
  
	close(s);

  return 0;
}

