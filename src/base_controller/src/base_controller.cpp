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
#define WHEELS_DISTANCE 0.090 //wheels distance is 90mm
#define WHEELS_RADIUS 0.0205 //the wheel radius is 20.5 mm
    
#define SRV_IP "192.168.1.2" //Put the IP of the Khepera you wish to connect

struct sockaddr_in si_other;//server adress
int s; //socket id


void chatterCallback(const geometry_msgs::Twist cmd_msg){
	
	static int i=0;
	float speedRight=0, speedLeft=0;
	char buf[BUFLEN];
	
	speedLeft=(2*cmd_msg.linear.x-cmd_msg.angular.z*WHEELS_DISTANCE)/2; //linear speed of the Left wheel
	speedRight=(2*cmd_msg.linear.x+cmd_msg.angular.z*WHEELS_DISTANCE)/2; //linear speed of the Right wheel

	//printf("L: %f, R: %f\n",speedLeft , speedRight);

	fflush(stdin);
	snprintf(buf, BUFLEN, "L%f R%f", speedLeft, speedRight); //puts the string in buf 
	if (sendto(s, buf, BUFLEN, 0, (const struct sockaddr*) &si_other, sizeof(si_other))==-1)
		//Using socket s, it sends the string buf of size BUFLEN to the server. 
		//Flags: 0
		//si_other is the struct with the server adress, the size of which is sizeof(si_other)
		exit(1);
		
	i++;
	puts(buf);

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
    
	ros::init(argc, argv, "base_controller");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("cmd_vel", 100, chatterCallback);
	
	ros::spin();
  
	close(s);

  return 0;
}

