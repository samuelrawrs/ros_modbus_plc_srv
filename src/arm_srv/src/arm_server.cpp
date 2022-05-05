#include "ros/ros.h"
#include "arm_srv/arm.h"
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>


int arm_position_val;
int arm_moving_state_val;

bool arm(arm_srv::arm::Request  &req,
         arm_srv::arm::Response &res)
{
//   ROS_INFO("rosservice request: %ld", (int)req.arm_request);
  if ((int)req.arm_request == 0){
    ROS_INFO("Client request: Brake");
  }
  else if ((int)req.arm_request == 1){
    ROS_INFO("Client request: Move Down");
  }
  else if ((int)req.arm_request == 2){
    ROS_INFO("Client request: Move Up");
  }
  else if ((int)req.arm_request == 3){
    ROS_INFO("Client request: Check Status");
  }
  else{
    ROS_INFO("arm_srv server error: int not recognised");
  }

//   ROS_INFO("Arm_position: %ld", (int)res.arm_position);
//   ROS_INFO("Arm_moving_state: %ld", (int)res.arm_moving_state);
    res.arm_position = arm_position_val;
    res.arm_moving_state = arm_moving_state_val;
  return true;
}

//void arm_position_Callback(const std_msgs::String::ConstPtr& msg)
void arm_position_Callback(const std_msgs::Int32 & msg)
{
//   arm_position_val = std::stoi(msg->data.c_str());
  arm_position_val = msg.data;
  if (arm_position_val == 1){
    ROS_INFO("Arm position: In between");
  }
  else if (arm_position_val == 0){
    ROS_INFO("Arm position: Bottom");
  }
  else if (arm_position_val == 2){
    ROS_INFO("Arm position: Top");
  }
  else if (arm_position_val == -1){
    ROS_INFO("Arm position: Unknown");
  }
  else{
    ROS_INFO("arm_srv server callback error");
  }
}

void arm_moving_state_Callback(const std_msgs::Int32 & msg)
{
  arm_moving_state_val = msg.data;
  if (arm_moving_state_val == 1){
    ROS_INFO("Moving State: Going Down");
  }
  else if (arm_moving_state_val == 0){
    ROS_INFO("Moving State: Brake");
  }
  else if (arm_moving_state_val == 2){
    ROS_INFO("Moving State: Going Up");
  }
  else if (arm_moving_state_val == -1){
    ROS_INFO("Arm position: Unknown");
  }
  else{
    ROS_INFO("arm_srv server callback error");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_server");
  ros::NodeHandle n;

  ros::Subscriber arm_position_sub = n.subscribe("arm_position", 1000, arm_position_Callback);
  ros::Subscriber arm_moving_state_sub = n.subscribe("arm_moving_state", 1000, arm_moving_state_Callback);

  ros::ServiceServer service = n.advertiseService("arm", arm);
  ROS_INFO("Ready to publish.");
  ros::spin();

  return 0;
}