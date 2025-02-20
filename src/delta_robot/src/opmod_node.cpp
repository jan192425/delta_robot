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
#include "delta_robot_interfaces/msg/op_mod.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include <kinematics.h>

using namespace std::chrono_literals;
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

 

    _subOpMod = this -> create_subscription<delta_robot_interfaces::msg::OpMod>("OpMod", 1, std::bind(&opmod::callbackMODE, this, _1));

    //_subGoalPosEffector = this -> create_subscription<delta_robot_interfaces::msg::GoalPosEffector>("GoalPosEffector",1, std::bind(&opmod::callbackPOINTNAV,this,_1));
    _pubGoalPosMotor = this -> create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position",QOS_RKL10V);

    _pubDirection = this -> create_publisher<geometry_msgs::msg::Twist>("MobRob",QOS_RKL10V);  //! topic name "MobRob" muss hier und beim MobRob gleichsein!! KA wie das topic beim mobrob heißt

    
    pos_client = this -> create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position");
    
    
    }

    private:
        void callbackMODE (const delta_robot_interfaces::msg::OpMod msg){
            //float xeffmsg = 0;
            //float yeffmsg = 0;
            //float zeffmsg = 0;
            //int point_before_compliant = 0; //TODO: indicates to compliant mode if pointnavigation was used, if thats the case compliant mode will use latest goalposition as refrence 

            switch (msg.mode){
                case 0:{
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
                        break;                
                }

                case 1:{
                     /*
                    for (int i=0; i<=2; i++){
                        auto req_motor_i = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
                        req_motor_i->id = i+1;

                        while (!pos_client->wait_for_service(1s)) {
                            if (!rclcpp::ok()) {
                            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                            }
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
                        }

                        fwdkin_in[i] = pos_client -> async_send_request(req_motor_i);
                    }*/
                    for (int i = 0; i <= 2; i++) {
                        auto req_motor_i = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
                        req_motor_i->id = i + 1;
                        
                        // Wait for service to be available
                        while (!pos_client->wait_for_service(1s)) {
                            if (!rclcpp::ok()) {
                                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                                return; // Exit if ROS is not ok
                            }
                            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
                        }
                        
                        auto result_future = pos_client->async_send_request(req_motor_i, 
                        [this,i](rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedFuture future) {
                            auto result = future.get();
                            //RCLCPP_INFO(this->get_logger(), "Position recieved [position: %d]", result->position);
                            fwdkin_in [i] = result -> position;
                        });
                        
                        
                        //fwdkin_in [i] = result -> position;
                        //RCLCPP_INFO(this->get_logger(), "Position recieved [position: %d]", fwdkin_in [i]);
                    }
                    
                    //RCLCPP_INFO(this->get_logger(), "Position recieved [position1: %d][position2: %d][position3: %d]", fwdkin_in [0], fwdkin_in [1], fwdkin_in [2]);

                    std::array<float, 3> fwdkinout = fwdkin(fwdkin_in[0], fwdkin_in[1], fwdkin_in[2]);
                                                           
                    //TODO:  Twist(linear)Nachricht veröffentlichen => lineargeschwindigkeiten x und y = deltx und delty (evtl. mal Verstärkungsfaktor) => Mobrob geschwindigkeit skaliert einfach mit Positionsabstand
                    geometry_msgs::msg::Twist twist;
                    twist.linear.x = fwdkinout[0];
                    twist.linear.y = fwdkinout[1];
                    _pubDirection -> publish(twist);
                    RCLCPP_INFO(this->get_logger(), "Twist published [linear x: %f] [linear y: %f] [linear z: %f]", fwdkinout[0], fwdkinout[1], fwdkinout[2]);
                    break;  
                }

         
            }
        }

    delta_robot_interfaces::msg::GoalPosEffector msg;

    dynamixel_sdk_custom_interfaces::msg::SetPosition out1;
    dynamixel_sdk_custom_interfaces::msg::SetPosition out2;
    dynamixel_sdk_custom_interfaces::msg::SetPosition out3;


    rclcpp::Subscription<delta_robot_interfaces::msg::GoalPosEffector>::SharedPtr _subGoalPosEffector;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr _pubGoalPosMotor;

    rclcpp::Subscription<delta_robot_interfaces::msg::OpMod>::SharedPtr _subOpMod;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _pubDirection;

    rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr pos_client;
    //rclcpp::Response<dynamixel_sdk_custom_interfaces::srv::GetPosition_Response>::SharedPtr result;
    //dynamixel_sdk_custom_interfaces::srv::GetPosition::Response::SharedPtr result;
    std::array<int, 3> fwdkin_in;
    
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









/*
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
*/





















//Walk-the dog 
// richtung 
// Effektor auslenken => Winkelveränderung erzeugt => regstirere Winkel => forward Kinematics um neuen Punkt zu berchenen => Lage punkt neu zu Punkt alt = Richtung 
// von ausgelenkter Position wieder zurück in Ursprung => Inverse Kinematik? oder nicht notwendig da einfach winkel auf 0 zurückfahren lasen
