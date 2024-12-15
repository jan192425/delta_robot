#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include <array>

#include "dynamixel_sdk/dynamixel_sdk.h"

#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

#include "delta_robot_interfaces/msg/goal_pos_effector.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include <kinematics.h>

using std::placeholders::_1;

bool initialize = false;

class opmod : public rclcpp::Node 
{
    public:
    opmod() : Node("OPMOD")
    {
    RCLCPP_INFO(this->get_logger(), "Run COMMOT node");

        //Set rclpp Qualtiy of Servie parameters: depth, reliable mode and durabiltiy to volatile(=no storage)
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);
    const auto QOS_RKL10V =
        rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    _pubGoalPosMotor = this -> create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position",QOS_RKL10V);
    _subGoalPosEffector = this -> create_subscription<delta_robot_interfaces::msg::GoalPosEffector>("GoalPosEffector",1, std::bind(&opmod::callbackPOINTNAV,this,_1));
  


    }

    private:
        void callbackPOINTNAV (const delta_robot_interfaces::msg::GoalPosEffector msg){
            
            float xeffmsg = msg.xeff;
            float yeffmsg = msg.yeff;
            float zeffmsg = msg.zeff;

            int (&invkinout)[3][2] = invkin(xeffmsg,yeffmsg, zeffmsg);

            out1.id         =invkinout [0][0];
            out1.position   =invkinout [0][1];
            out2.id         =invkinout [1][0];
            out2.position   =invkinout [1][1];
            out3.id         =invkinout [2][0];
            out3.position   =invkinout [2][1];
            

            _pubGoalPosMotor -> publish(out1);
            _pubGoalPosMotor -> publish(out2);
            _pubGoalPosMotor -> publish(out3);
        
        }

    delta_robot_interfaces::msg::GoalPosEffector msg;

    dynamixel_sdk_custom_interfaces::msg::SetPosition out1;
    dynamixel_sdk_custom_interfaces::msg::SetPosition out2;
    dynamixel_sdk_custom_interfaces::msg::SetPosition out3;


    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr _pubGoalPosMotor;
    rclcpp::Subscription<delta_robot_interfaces::msg::GoalPosEffector>::SharedPtr _subGoalPosEffector;

};


int main (int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto opmod_start = std::make_shared<opmod>();
    rclcpp::spin(opmod_start);
    rclcpp::shutdown();
    return 0;
}

































//Walk-the dog 
// richtung 
// Effektor auslenken => Winkelveränderung erzeugt => regstirere Winkel => forward Kinematics um neuen Punkt zu berchenen => Lage punkt neu zu Punkt alt = Richtung 
// von ausgelenkter Position wieder zurück in Ursprung => Inverse Kinematik? oder nicht notwendig da einfach winkel auf 0 zurückfahren lasen
