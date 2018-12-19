#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <fstream>

#define KEYCODE_UP 0x77 //w
#define KEYCODE_DOWN 0x78 //x
#define KEYCODE_LEFT 0x61 //a
#define KEYCODE_RIGHT 0x64 //d

#define KEYCODE_UP_LEFT 0x71 //q
#define KEYCODE_DOWN_LEFT 0x7A //z
#define KEYCODE_UP_RIGHT 0x65 //e
#define KEYCODE_DOWN_RIGHT 0x63 //c

#define KEYCODE_STOP 0x73 //s

#define KEYCODE_SPEED_UP 0x6A //j
#define KEYCODE_SPEED_DOWN 0x6B //k

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_, speed;
  ros::Publisher twist_pub_;
  
};

TeleopTurtle::TeleopTurtle():
  speed(1),
  linear_(0),
  angular_(0),
  l_scale_(0.1),
  a_scale_(0.5)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  
  return(0);
}


void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the robot.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_INFO("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_LEFT:
        ROS_INFO("LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        ROS_INFO("RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_UP:
        ROS_INFO("UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_DOWN:
        ROS_INFO("DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_UP_LEFT:
        ROS_INFO("UP_LEFT");
        linear_ = 1.0;
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_DOWN_LEFT:
        ROS_INFO("DOWN_LEFT");
        linear_ = -1.0;
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_UP_RIGHT:
        ROS_INFO("UP_RIGHT");
        linear_ = 1.0;
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_DOWN_RIGHT:
        ROS_INFO("DOWN_RIGHT");
        linear_ = -1.0;
        angular_ = 1.0;
        dirty = true;
        break;
	    case KEYCODE_SPEED_DOWN:
        ROS_INFO("SPEED DOWN");
        speed = speed - 0.1;
        dirty = true;
        break;
	    case KEYCODE_SPEED_UP:
        ROS_INFO("SPEED UP");
        speed = speed + 0.1;
        dirty = true;
        break;
      case KEYCODE_STOP:
        ROS_INFO("STOP");
        linear_ = 0;
        angular_ = 0;
        dirty = true;
        break;
    }
   
    if(speed > 4) {
      speed = 4;
    } else if (speed < 0) {
      speed = 0;
    }    

    geometry_msgs::Twist twist;
    twist.angular.z = speed*a_scale_*angular_;
    twist.linear.x = speed*l_scale_*linear_;
    ROS_INFO("Linear: %f", twist.linear.x);
    ROS_INFO("Angular: %f", twist.angular.z);
    ROS_INFO("Speed: %f", speed);
    if(dirty ==true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
      saveCommandInFile(twist);
    }
  }


  return;
}
