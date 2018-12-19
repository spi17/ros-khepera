#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#define DEFAULT_AXIS_LINEAR_X	    1
#define DEFAULT_AXIS_ANGULAR_Z		0
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		8.0

#define WHEELS_DISTANCE 0.230 //wheels distance is 90mm
#define WHEELS_RADIUS 0.025 //the wheel radius is 20.5 mm

#define MAX_SPEED 0.5 //maximum speed 0.5 m/s

class KheperaJoy
{
public:
  KheperaJoy();

private:
  void joyCallback(const geometry_msgs::Twist vel);  
  
  ros::NodeHandle nh_;
  
  int linear_x, angular_z;
  double l_scale_, a_scale_;  
  
  ros::Publisher vel_pub_; //! It will publish into command velocity (for the robot) and 
 
  ros::Subscriber twist_sub_;//! It will be suscribed to the joystick 

  std::string cmd_topic_vel_;  //! Name of the topic where it will be publishing the velocity 
 
};

KheperaJoy::KheperaJoy()
{
  	// Publish through the node handle Joy type messages to the Unity model
	// vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);
	vel_pub_ = nh_.advertise<sensor_msgs::Joy>("joy_topic", 5);
	//Publish to topic

 	 // Listen through the node handle sensor_msgs::Joy messages from joystick (these are the orders that we will send to the Unity model)
	twist_sub_ = nh_.subscribe("cmd_vel", 10, &KheperaJoy::joyCallback, this);
	//Ask for services
	
}

void KheperaJoy::joyCallback(const geometry_msgs::Twist vel)
{
	sensor_msgs::Joy joy_output;

	float speedRight=0, speedLeft=0;

	// Los sentidos no estan claros
	
		speedLeft=(((2*vel.linear.x) - vel.angular.z) * WHEELS_DISTANCE) / 2; //linear speed of the Left wheel
		speedRight=(((2*vel.linear.x) + vel.angular.z) * WHEELS_DISTANCE) / 2; //linear speed of the Right wheel

		joy_output.axes.push_back(speedLeft);
		joy_output.axes.push_back(speedRight);

		ROS_INFO("Velocity: %f", joy_output.axes[0]);	
		ROS_INFO("Velocity: %f", joy_output.axes[1]);	
		
		
		vel_pub_.publish(joy_output);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "Khepera_GamePad");
	KheperaJoy khepera_joy;

	ros::spin();
}

