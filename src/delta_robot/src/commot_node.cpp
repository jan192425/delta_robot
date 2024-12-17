/*
#include <iostream>
#include <chrono>
#include <functional>
*/
#include <memory>
#include <string>
#include <cstdio>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "../include/commot_node.hpp"


// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  //! [Linux]: "/dev/ttyUSBXXXXXXXX", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

commot::commot(): Node("COMMOT")
{
    //Test if Node is active
    RCLCPP_INFO(this->get_logger(), "Run COMMOT node");

    //Set rclpp Qualtiy of Servie parameters: depth, reliable mode and durabiltiy to volatile(=no storage)
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);
    const auto QOS_RKL10V =
        rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
    
    //?Goal Position setting and logging (topic based)
    set_position_subscriber_ =
    this->create_subscription<SetPosition>(
    "set_position",
    QOS_RKL10V,
    [this](const SetPosition::SharedPtr msg) -> void  //lambda function with capture of local class and void {functionality} return
    {
      uint8_t dxl_error = 0;

      // Position Value of X series is 4 byte data.
      // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
      uint32_t goal_position = (unsigned int)msg->position;  // Convert int32 -> uint32

      //* Write Goal Position (length : 4 bytes)
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      dxl_comm_result =
      packetHandler->write4ByteTxRx(    //write&read functions can be found in DynamixelSDK/dynamixel_sdk/include/dynamixel_sdk/protocol2_packet_handler.h
        portHandler,
        (uint8_t) msg->id,
        ADDR_GOAL_POSITION,
        goal_position,
        &dxl_error
      );
      //*=====> after writing the motor will move to goal position (if motor in position control mode)

      //Definition of logger output
      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
      }
    }
    );

    //?Reading and logging of current postion (service based) => "simple callback via lambda function"
    auto get_present_position =
      [this](
      const std::shared_ptr<GetPosition::Request> request,
      std::shared_ptr<GetPosition::Response> response) -> void
      {
        // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
        // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
        dxl_comm_result = packetHandler->read4ByteTxRx(
          portHandler,
          (uint8_t) request->id,
          ADDR_PRESENT_POSITION,
          reinterpret_cast<uint32_t *>(&present_position),
          &dxl_error
        );

        RCLCPP_INFO(
          this->get_logger(),
          "Get [ID: %d] [Present Position: %d]",
          request->id,
          present_position
        );

        response->position = present_position;
        /*
        //publish position to OPMOD node via Publisher
        msg_typ::mgs::.... //!vllt einfach set postion message type verwenden?
        irgendwie message = present_postion
        */
      };

    //create a service sever for the defined lambda function
    get_position_server_ = create_service<GetPosition>("get_position", get_present_position);
    
    /*
    //?publisher for of the current motor position to OPMOD Node
    pospub = this->create_publisher<pub TYP>("topicname", queue size)
    */
}

commot::~commot()
{
}

void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    3,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("COMMOT"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("COMMOT"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(  
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("COMMOT"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("COMMOT"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("COMMOT"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("COMMOT"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("COMMOT"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("COMMOT"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto commot_start = std::make_shared<commot>();
  rclcpp::spin(commot_start);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}


//TODO: Herausfinden wie man IDs den einzelnen Motoren zuweist und seperat über COMMOT mit ihnen kommunizieren kann
//TODO: alles aus DelRobCode in github hochladen
//TODO: dynamixel sdk für humble -package in delrob verzeichnis installieren und colcon build mal testweise durchlaufen lassen
/*
using namespace std::chrono_literals;

class commot : public rclpp::Node
{

    public:
        commot():Node("COMMOT")
        {
            
        }


}

theta1 = currentpostion1 *0.088*; //Unit: deg | currentpostion1 = Platzhalter für adresscall von Motor1 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<commot>());
  rclcpp::shutdown();
  return 0;
}*/