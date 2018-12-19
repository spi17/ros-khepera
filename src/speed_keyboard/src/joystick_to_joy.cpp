#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#define DEFAULT_NUM_OF_BUTTONS		10
#define DEFAULT_AXIS_LINEAR_X	    1
#define DEFAULT_AXIS_ANGULAR_Z		0
#define DEFAULT_SCALE_LINEAR		1.0
#define DEFAULT_SCALE_ANGULAR		8.0
#define NUM_BUTTONS                 12

#define WHEELS_DISTANCE 0.230 //wheels distance is 90mm
#define WHEELS_RADIUS 0.025 //the wheel radius is 20.5 mm

class KheperaJoy
{
public:
  KheperaJoy();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);  
  ros::NodeHandle nh_;
  int linear_x, angular_z;
  double l_scale_, a_scale_;  
  ros::Publisher vel_pub_; //! It will publish into command velocity (for the robot) and 
 
  ros::Subscriber joy_sub_;//! It will be suscribed to the joystick 

  std::string cmd_topic_vel_;  //! Name of the topic where it will be publishing the velocity 
  double current_vel;    	
  int speed_up_button_, speed_down_button_; //! Number of the button for increase or decrease the speed max of the joystick
  int num_of_buttons_; //! Number of buttons of the joystick  
  bool bRegisteredButtonEvent[NUM_BUTTONS]; //! Pointer to a vector for controlling the event when pushing the buttons
};

KheperaJoy::KheperaJoy()
{

	current_vel = 0.5;
	
	nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
	// MOTION CONF
	nh_.param("axis_linear_x", linear_x, DEFAULT_AXIS_LINEAR_X);
	nh_.param("axis_angular_z", angular_z, DEFAULT_AXIS_ANGULAR_Z);
	
	nh_.param("scale_angular", a_scale_, DEFAULT_SCALE_ANGULAR);
	nh_.param("scale_linear", l_scale_, DEFAULT_SCALE_LINEAR);
	nh_.param("cmd_topic_vel", cmd_topic_vel_, std::string("cmd_vel"));
	nh_.param("button_speed_up", speed_up_button_, 3);  //4 Thrustmaster
	nh_.param("button_speed_down", speed_down_button_, 0); //5 Thrustmaster
	//
	for(int i = 0; i < NUM_BUTTONS; i++){
		bRegisteredButtonEvent[i] = false;
	}

	ROS_INFO("Service I/O = [%s]", cmd_topic_vel_.c_str());
	ROS_INFO("Axis linear_x = %d", linear_x);
	ROS_INFO("Axis angular_z = %d", angular_z);
	ROS_INFO("Scale angular = %f", a_scale_);

  	// Publish through the node handle Joy type messages to the Unity model
	// vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_topic_vel_, 1);
	vel_pub_ = nh_.advertise<sensor_msgs::Joy>("joy_topic", 5);
	//Publish to topic

 	 // Listen through the node handle sensor_msgs::Joy messages from joystick (these are the orders that we will send to the Unity model)
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &KheperaJoy::joyCallback, this);
	//Ask for services
	
}

void KheperaJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	//static int i=0;
	geometry_msgs::Twist vel;
	sensor_msgs::Joy joy_output;
	float speedRight=0, speedLeft=0;
	//tf::TransformBroadcaster broadcaster;
	//broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.0, -0.06)), ros::Time::now(),"/base_link", "/base_laser"));

	
	    if ( joy->buttons[speed_down_button_] == 1 )
	    {
	    	if(!bRegisteredButtonEvent[speed_down_button_])
	    	{
	    		if(current_vel > 0.1)
	    		{
	    			current_vel = current_vel - 0.1;
	    			bRegisteredButtonEvent[speed_down_button_] = true;
					ROS_INFO("Velocity: %f%%", current_vel*100.0);	
				}
	    	}
		}
	    else{
	    	bRegisteredButtonEvent[speed_down_button_] = false;
	    }

  		if (joy->buttons[speed_up_button_] == 1)
  		{
			if(!bRegisteredButtonEvent[speed_up_button_])
			{
				if(current_vel < 0.9)
				{
					current_vel = current_vel + 0.1;
					bRegisteredButtonEvent[speed_up_button_] = true;
			 	 	ROS_INFO("Velocity: %f%%", current_vel*100.0);
				}
			}
		}
		else{
			bRegisteredButtonEvent[speed_up_button_] = false;
		}
		 
		vel.angular.z = current_vel*(a_scale_*joy->axes[angular_z]);
		vel.linear.x = current_vel*l_scale_*joy->axes[linear_x];

		speedLeft=(((2*vel.linear.x) - vel.angular.z) * WHEELS_DISTANCE) / 2; //linear speed of the Left wheel
		speedRight=(((2*vel.linear.x) + vel.angular.z) * WHEELS_DISTANCE) / 2; //linear speed of the Right wheel

		joy_output.axes[0] = speedLeft;
		joy_output.axes[1] = speedRight;
		
		vel_pub_.publish(joy_output);

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "Khepera_GamePad");
	KheperaJoy khepera_joy;

	ros::spin();
}

