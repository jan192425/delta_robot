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
        


    }

    private:

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
