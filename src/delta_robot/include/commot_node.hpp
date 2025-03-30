
#ifndef COMMOT_NODE_HPP
#define COMMOT_NODE_HPP

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

#include "delta_robot_interfaces/msg/op_mod.hpp"

class commot : public rclcpp::Node
{
public:
  using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
  using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

  commot();
  virtual ~commot(); 

private:
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  rclcpp::Subscription<delta_robot_interfaces::msg::OpMod>::SharedPtr OpMod_subscriber_;
  
  int present_position;
  int present_position_i;
};

#endif  