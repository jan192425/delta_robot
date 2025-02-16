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
    RCLCPP_INFO(this->get_logger(), "Run OPMOD node");
    auto logger = this->get_logger();
        //Set rclpp Qualtiy of Servie parameters: depth, reliable mode and durabiltiy to volatile(=no storage)
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);
    const auto QOS_RKL10V =
        rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

 

    
    _subGoalPosEffector = this -> create_subscription<delta_robot_interfaces::msg::GoalPosEffector>("GoalPosEffector",1, std::bind(&opmod::callbackPOINTNAV,this,_1));
    
    _pubGoalPosMotor = this -> create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position",QOS_RKL10V);
    
    
    //! Idee adaptive Regelparameter als Funktion der Regelabweichung => Problem muss das in idealerweise in read/write Node machen => sonst rel redundante node struktur und keine klare trennung


    }

    private:
        void callbackPOINTNAV (const delta_robot_interfaces::msg::GoalPosEffector msg){
            //Definition of logger output
            RCLCPP_INFO(this->get_logger(), "Recieved Goal Position [xeff: %f] [yeff: %f] [zeff: %f]", msg.xeff, msg.yeff, msg.zeff);
            
            float xeffmsg = msg.xeff;
            float yeffmsg = msg.yeff;
            float zeffmsg = -msg.zeff; //kinematics is based on a normal delta robot => needs a neg. z coord but in this case it's more intuitive to set a positive z coord

            

            int (&invkinout)[3][3] = invkin(xeffmsg,yeffmsg, zeffmsg);
             RCLCPP_INFO(this->get_logger(), "Calculated Motor Angles [xeff: %d] [yeff: %d] [zeff: %d]", invkinout [0][2], invkinout [1][2], invkinout [2][2]);
             RCLCPP_INFO(this->get_logger(), "Calculated Motor Positions [xeff: %d] [yeff: %d] [zeff: %d]", invkinout [0][1], invkinout [1][1], invkinout [2][1]);

            out1.id         =invkinout [0][0];
            out1.position   =invkinout [0][1];
            out2.id         =invkinout [1][0];
            out2.position   =invkinout [1][1];
            out3.id         =invkinout [2][0];
            out3.position   =invkinout [2][1];
            

            _pubGoalPosMotor -> publish(out1);
            _pubGoalPosMotor -> publish(out2);
            _pubGoalPosMotor -> publish(out3);

            RCLCPP_INFO(this->get_logger(), "Calculated Motor Positions [ID: %d] [Position: %d] [ID: %d] [Position: %d] [ID: %d] [Position: %d]", out1.id , out1.position , out2.id, out2.position, out3.id, out3.position);
        
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
    /**/
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
