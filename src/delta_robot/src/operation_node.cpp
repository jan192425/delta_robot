#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"


#include <kinematics.h>

bool initialize = false;

class opmod : public rclcpp:Node 
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

    _pubGoalPosMotor= this -> create_publisher<SetPosition>("set_position",QOS_RKL10V);
        
        
        std::bind(&opmod::callbackPOINTNAV,this,_1))


    }

    private:


    rclcpp::Publisher<SetPosition>::SharedPtr _pubGoalPosMotor;

}



//Point nav (inversekin) 
//!subscriber für zielposition endefftor => msg type für diese Position selbst definiern 

//!Publisher
//=> einfach berechnete Winkel auf set postion PUBLISHEN für jeden motor mit seiner eigenen ID => wird dann von commot node an motoren geschriebe
 this->create_subscription<SetPosition>(
    "set_position",
    QOS_RKL10V, 






































//Walk-the dog 
// richtung 
// Effektor auslenken => Winkelveränderung erzeugt => regstirere Winkel => forward Kinematics um neuen Punkt zu berchenen => Lage punkt neu zu Punkt alt = Richtung 
// von ausgelenkter Position wieder zurück in Ursprung => Inverse Kinematik? oder nicht notwendig da einfach winkel auf 0 zurückfahren lasen
