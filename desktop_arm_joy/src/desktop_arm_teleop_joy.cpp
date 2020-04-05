#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <math.h>
#include <sstream>

class RobotArm
{
public:
  RobotArm();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;

  ros::Publisher chatter_pub_;
  ros::Publisher servo_pub_;
  ros::Subscriber joy_sub_;

};


RobotArm::RobotArm():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  servo_pub_ = nh_.advertise<std_msgs::UInt16>("servo", 1);
  chatter_pub_ = nh_.advertise<std_msgs::String>("chatter",1000);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RobotArm::joyCallback, this);

}

void RobotArm::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  std_msgs::String msg;
  std_msgs::UInt16 angle;
  float  x, y, new_pos;

  y = a_scale_*joy->axes[angular_];
  x = l_scale_*joy->axes[linear_];
  new_pos = atan2(-y,x)*(180/(3.141592));
  angle.data = -new_pos;
  //std::stringstream ss << "a_scale_ " << a_scale_ << " | l_scale_ " << l_scale << " | angular_" << angular_ << " | linear_" << linear_ << " | y_angle" << y_angle << " | x_angle " << x_angle;
  std::stringstream ss;
  ss << "y=" << y << " | x=" << x << "new pos" << new_pos;
  msg.data = ss.str();

  //ROS_INFO("%s", msg.data.c_str());

  chatter_pub_.publish(msg);
  servo_pub_.publish(angle);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_desktop_arm");

  RobotArm teleop_desktop_arm;

  ros::spin();
}
