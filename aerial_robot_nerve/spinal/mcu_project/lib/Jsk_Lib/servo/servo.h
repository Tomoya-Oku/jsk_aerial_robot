/**
******************************************************************************
* File Name          : servo.h
* Description        : universal servo control interface for Spinal
* Author             : J.Sugihara (2024/3/1)
includes ------------------------------------------------------------------*/


#ifndef APPLICATION_SERVO_TEMP_SERVO_H_
#define APPLICATION_SERVO_TEMP_SERVO_H_

#include "drivers/Dynamixel/dynamixel_serial.h"
#include "drivers/kondo_servo/kondo_servo.h"
#include <ros.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/DirectServoCurrent.h>
#include <spinal/DirectServoMode.h>
#include <spinal/ServoStates.h>
#include <spinal/ServoTorqueStates.h>
#include <spinal/ServoTorqueCmd.h>
#include <spinal/SetDirectServoConfig.h>
#include <spinal/GetBoardInfo.h>
#include <spinal/JointProfiles.h>
#include <string.h>
#include <config.h>
#include <map>
#include "flashmemory/flashmemory.h"

class Initializer;

namespace ValueType
{
  enum
    {BIT = 0, RADIAN = 1};
};

class DirectServo
{
public:
  enum class Arm {Legacy, Right, Left};
  explicit DirectServo(Arm arm = Arm::Legacy):
    servo_ctrl_sub_("servo/target_states", &DirectServo::servoControlCallback,this),
    servo_torque_ctrl_sub_("servo/torque_enable", &DirectServo::servoTorqueControlCallback,this),
    direct_current_sub_(endpoint(arm, "servo/direct_current", "right/servo/direct_current",
                                 "left/servo/direct_current"), &DirectServo::directCurrentCallback,this),
    direct_mode_srv_(endpoint(arm, "direct_servo_mode", "right/direct_servo_mode",
                              "left/direct_servo_mode"), &DirectServo::directModeCallback,this),
    joint_profiles_sub_("joint_profiles", &DirectServo::jointProfilesCallback,this),
    servo_state_pub_(endpoint(arm, "servo/states", "right/servo/states", "left/servo/states"),
                     &servo_state_msg_),
    servo_torque_state_pub_(endpoint(arm, "servo/torque_states", "right/servo/torque_states",
                                    "left/servo/torque_states"), &servo_torque_state_msg_),
    servo_config_srv_("direct_servo_config", &DirectServo::servoConfigCallback, this),
    board_info_srv_(endpoint(arm, "get_board_info", "right/get_board_info", "left/get_board_info"),
                    &DirectServo::boardInfoCallback,this)
  {
  }
  ~DirectServo(){}

  bool init(UART_HandleTypeDef* huart, ros::NodeHandle* nh, osMutexId* mutex);
#if DYNAMIXEL && !KONDO
  bool initRs485(UART_HandleTypeDef* huart, ros::NodeHandle* nh, osMutexId* mutex,
                 uint8_t* rx_buffer, uint8_t first_id, uint8_t count);
#endif
  void update();
  bool connected() {return connected_;}
  void sendData(bool flag_send_asap);
  void torqueEnable(const std::map<uint8_t, float>& servo_map);
  void setGoalAngle(const std::map<uint8_t, float>& servo_map, uint8_t value_type = 0);
#if KONDO
  KondoServo& getServoHnadler() {return servo_handler_;}
#else DYNAMIXEL
  DynamixelSerial& getServoHnadler() {return servo_handler_;}  
#endif

  uint32_t rad2Pos(float angle, float scale, uint32_t zero_point_pos){
    return static_cast<uint32_t>(angle /scale + zero_point_pos);
  }

private:
  static const char* endpoint(Arm arm, const char* legacy, const char* right, const char* left)
  {
    return arm == Arm::Right ? right : arm == Arm::Left ? left : legacy;
  }
  bool registerRos(ros::NodeHandle* nh);
  void sendRs485Data();
  bool fixed_rs485_ = false;

  /* ROS */
  ros::NodeHandle* nh_;
  ros::Subscriber<spinal::ServoControlCmd, DirectServo> servo_ctrl_sub_;
  ros::Subscriber<spinal::ServoTorqueCmd, DirectServo> servo_torque_ctrl_sub_;
  ros::Subscriber<spinal::JointProfiles, DirectServo> joint_profiles_sub_;
  ros::Subscriber<spinal::DirectServoCurrent,DirectServo> direct_current_sub_;
  ros::ServiceServer<spinal::DirectServoMode::Request,spinal::DirectServoMode::Response,DirectServo> direct_mode_srv_;
  void directCurrentCallback(const spinal::DirectServoCurrent& msg);
  void directModeCallback(const spinal::DirectServoMode::Request& req,spinal::DirectServoMode::Response& res);
  ros::Publisher servo_state_pub_;
  ros::Publisher servo_torque_state_pub_;

  ros::ServiceServer<spinal::SetDirectServoConfig::Request, spinal::SetDirectServoConfig::Response, DirectServo> servo_config_srv_;
  ros::ServiceServer<spinal::GetBoardInfo::Request, spinal::GetBoardInfo::Response, DirectServo> board_info_srv_;

  spinal::ServoStates servo_state_msg_;
  spinal::ServoTorqueStates servo_torque_state_msg_;
  spinal::GetBoardInfo::Response board_info_res_;

  uint32_t servo_last_pub_time_;
  uint32_t servo_torque_last_pub_time_;

  void servoControlCallback(const spinal::ServoControlCmd& control_msg);
  void servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg);
  void jointProfilesCallback(const spinal::JointProfiles& joint_prof_msg);
  
  void servoConfigCallback(const spinal::SetDirectServoConfig::Request& req, spinal::SetDirectServoConfig::Response& res);
  void boardInfoCallback(const spinal::GetBoardInfo::Request& req, spinal::GetBoardInfo::Response& res);
  
  /* Servo state */
  struct ServoState{
    int16_t angle;
    uint8_t temperature;
    uint8_t moving;
    int16_t current;
    uint8_t error;
    ServoState(uint16_t angle, uint8_t temperature, uint8_t moving, int16_t current, uint8_t error)
      :angle(angle), temperature(temperature), moving(moving), current(current), error(error){}
  };

  struct JointProf{
    uint8_t servo_id;
    int8_t angle_sgn;
    float angle_scale;
    int16_t zero_point_offset;
  };

  JointProf joint_profiles_[MAX_SERVO_NUM];

#if KONDO
  KondoServo servo_handler_;
#else DYNAMIXEL
  DynamixelSerial servo_handler_;
#endif

  bool connected_ = false;

  friend class Initializer;
};


#endif /* APPLICATION_SERVO_SERVO_H_ */
