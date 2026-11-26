/**
******************************************************************************
* File Name          : servo.cpp
* Description        : universal servo control interface for Spinal
* Author             : J.Sugihara (2024/3/1)
 ------------------------------------------------------------------*/


#include "servo.h"

#define  SERVO_PUB_INTERVAL 20 // 50Hz
#define SERVO_TORQUE_PUB_INTERVAL  1000 // 1Hz

bool DirectServo::init(UART_HandleTypeDef* huart,  ros::NodeHandle* nh, osMutexId* mutex = NULL) //TODO: support encoder
{
  /*setup pin configuration*/
#if !STM32H7_V2
#ifdef STM32H7
  uint32_t raw_baudrate = huart->Init.BaudRate;
  HAL_UART_DeInit(huart);
  huart->Init.BaudRate = 1000000;
  HAL_UART_Init(huart);
#endif
#endif

  /* initialize */
  servo_handler_.init(huart, mutex);
  unsigned int actual_servo_num = servo_handler_.getServoNum();

  if (actual_servo_num == 0) {
#if !STM32H7_V2
#ifdef STM32H7
    HAL_UART_DeInit(huart);
    huart->Init.BaudRate = raw_baudrate;
    HAL_UART_Init(huart);
#endif
#endif
    connected_ = false;
    return false;
  }

  return registerRos(nh);
}

#if DYNAMIXEL && !KONDO
bool DirectServo::initRs485(UART_HandleTypeDef* huart, ros::NodeHandle* nh, osMutexId* mutex,
                          uint8_t* rx_buffer, uint8_t first_id, uint8_t count)
{
  fixed_rs485_ = true;
  return servo_handler_.initRs485(huart, mutex, rx_buffer, first_id, count) && registerRos(nh);
}
#endif

bool DirectServo::registerRos(ros::NodeHandle* nh)
{
  nh_ = nh;
  const unsigned int actual_servo_num = servo_handler_.getServoNum();
  if (!fixed_rs485_) {
    nh_->subscribe(servo_ctrl_sub_);
    nh_->subscribe(servo_torque_ctrl_sub_);
    nh_->subscribe(joint_profiles_sub_);
    nh_->advertiseService(servo_config_srv_);
  }
#if !KONDO
  nh_->subscribe(direct_current_sub_);
  nh_->advertiseService(direct_mode_srv_);
#endif
  nh_->advertise(servo_state_pub_);
  nh_->advertise(servo_torque_state_pub_);
  nh_->advertiseService(board_info_srv_);

  servo_state_msg_.servos_length = actual_servo_num;
  servo_state_msg_.servos = new spinal::ServoState[actual_servo_num];
  servo_torque_state_msg_.torque_enable_length = actual_servo_num;
  servo_torque_state_msg_.torque_enable = new uint8_t[actual_servo_num];

  servo_last_pub_time_ = 0;
  servo_torque_last_pub_time_ = 0;

  board_info_res_.boards_length = 1;
  board_info_res_.boards = new spinal::BoardInfo[1];
  board_info_res_.boards[0].servos_length = servo_handler_.getServoNum();
  board_info_res_.boards[0].servos = new spinal::ServoInfo[servo_handler_.getServoNum()];

  connected_ = true;
  return true;
}

void DirectServo::update()
{
  servo_handler_.update();
  if (fixed_rs485_) sendRs485Data();
  else sendData(true);
}

void DirectServo::sendRs485Data()
{
#if DYNAMIXEL && !KONDO
  if (servo_handler_.getROSCommFlag()) {
    servo_handler_.setROSCommFlag(false);
    unsigned length = 0;
    for (unsigned i = 0; i < servo_handler_.getServoNum(); ++i) {
      if (!servo_handler_.positionReceived(i)) continue;
      const ServoData& servo = servo_handler_.getServo()[i];
      auto& state = servo_state_msg_.servos[length++];
      state.index = i;
      state.angle = servo.present_position_;
      state.temp = servo.present_temp_;
      state.load = servo.present_current_;
      state.error = servo.hardware_error_status_;
    }
    // Do not timestamp cached positions as new measurements after a bus failure.
    if (length) {
      // ServoStates has one stamp for the whole sweep. Use its start so early
      // samples cannot appear newer while waiting for later, unresponsive axes.
      const uint32_t age_ms = HAL_GetTick() - servo_handler_.positionSweepStart();
      servo_state_msg_.stamp = nh_->now();
      servo_state_msg_.stamp -= ros::Duration(age_ms / 1000, (age_ms % 1000) * 1000000);
      servo_state_msg_.servos_length = length;
      servo_state_pub_.publish(&servo_state_msg_);
    }
  }
  const uint32_t now = HAL_GetTick();
  if (now - servo_torque_last_pub_time_ >= SERVO_TORQUE_PUB_INTERVAL) {
    for (unsigned i = 0; i < servo_handler_.getServoNum(); ++i)
      servo_torque_state_msg_.torque_enable[i] = servo_handler_.getServo()[i].torque_enable_;
    servo_torque_state_pub_.publish(&servo_torque_state_msg_);
    servo_torque_last_pub_time_ = now;
  }
#endif
}

void DirectServo::sendData(bool flag_send_asap)
{
  uint32_t now_time = HAL_GetTick();

  if (flag_send_asap && servo_handler_.getROSCommFlag() == true)  // This setting will ignore the setting of SERVO_PUB_INTERVAL and pub the information once the measurement is updated.
  {
	  servo_state_msg_.stamp = nh_->now();
      for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
        const ServoData& s = servo_handler_.getServo()[i];
        if (s.send_data_flag_ != 0) {
          spinal::ServoState servo;
          servo.index = i;
          servo.angle = s.present_position_;
          servo.temp = s.present_temp_;
          servo.load = s.present_current_;
          servo.error = s.hardware_error_status_;
          servo_state_msg_.servos[i] = servo;
        }
      }
      servo_state_pub_.publish(&servo_state_msg_);
      servo_last_pub_time_ = now_time;

      servo_handler_.setROSCommFlag(false);
  }
  else
  {
  if( now_time - servo_last_pub_time_ >= SERVO_PUB_INTERVAL)
    {
	  servo_state_msg_.stamp = nh_->now();
      for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
        const ServoData& s = servo_handler_.getServo()[i];
        if (s.send_data_flag_ != 0) {
          spinal::ServoState servo;
          servo.index = i;
          servo.angle = s.present_position_;
          servo.temp = s.present_temp_;
          servo.load = s.present_current_;
          servo.error = s.hardware_error_status_;
          servo_state_msg_.servos[i] = servo;
        }
      }
      servo_state_pub_.publish(&servo_state_msg_);
      servo_last_pub_time_ = now_time;
    }
  }

  if( now_time - servo_torque_last_pub_time_ >= SERVO_TORQUE_PUB_INTERVAL)
    {
      for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
        const ServoData& s = servo_handler_.getServo()[i];
        if (s.send_data_flag_ != 0) {
          servo_torque_state_msg_.torque_enable[i] = s.torque_enable_;
        }
      }
      servo_torque_state_pub_.publish(&servo_torque_state_msg_);
      servo_torque_last_pub_time_= now_time;
    }  
}

void DirectServo::torqueEnable(const std::map<uint8_t, float>& servo_map)
{
  for (auto servo : servo_map)
    {
      JointProf joint_prof = joint_profiles_[servo.first];
      uint8_t index = servo.first;
      if(index >= servo_handler_.getServoNum())
        {
          nh_->logerror("Invalid Servo ID!");
          return;
        }
  #if !KONDO
    if (servo_handler_.currentManaged(index)) continue;
#endif
    ServoData& s = servo_handler_.getServo()[index];
      if(servo.second && !s.torque_enable_){
        s.torque_enable_ = true;
        servo_handler_.setTorque(index);
      }
      else if(!servo.second && s.torque_enable_){
        s.torque_enable_ = false;
        servo_handler_.setTorque(index);
      }     
    }
}

void DirectServo::setGoalAngle(const std::map<uint8_t, float>& servo_map, uint8_t value_type)
{
  for (auto servo : servo_map)
    {
      JointProf joint_prof = joint_profiles_[servo.first];
      int32_t goal_pos;
      if(value_type == ValueType::BIT){
        goal_pos = static_cast<int32_t>(servo.second);
      }else if(value_type == ValueType::RADIAN){
        goal_pos = static_cast<int32_t>(servo.second*joint_prof.angle_sgn/joint_prof.angle_scale + joint_prof.zero_point_offset);
      }

      uint8_t index = servo.first;
      if(index >= servo_handler_.getServoNum())
        {
          nh_->logerror("Invalid Servo ID!");
          return;
        }
  #if !KONDO
    if (servo_handler_.currentManaged(index)) continue;
#endif
    ServoData& s = servo_handler_.getServo()[index];
      s.setGoalPosition(goal_pos);
      if (! s.torque_enable_) {
        s.torque_enable_ = true;
        servo_handler_.setTorque(index);
      }
      
    }
}

void DirectServo::servoControlCallback(const spinal::ServoControlCmd& control_msg)
{
  if (control_msg.index_length != control_msg.angles_length) return;
  for (unsigned int i = 0; i < control_msg.index_length; i++) {
    uint8_t index = control_msg.index[i];
    if(index >= servo_handler_.getServoNum())
      {
        nh_->logerror("Invalid Servo ID!");
        return;
      }
#if !KONDO
    if (servo_handler_.currentManaged(index)) continue;
#endif
    ServoData& s = servo_handler_.getServo()[index];
    int32_t goal_pos = static_cast<int32_t>(control_msg.angles[i]);
    s.setGoalPosition(goal_pos);
    if (! s.torque_enable_) {
      s.torque_enable_ = true;
      servo_handler_.setTorque(index);
    }
  }
}

void DirectServo::servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg)
{
  if (control_msg.index_length != control_msg.torque_enable_length) return;
  for (unsigned int i = 0; i < control_msg.index_length; i++) {
    uint8_t index = control_msg.index[i];
    if(index >= servo_handler_.getServoNum())
      {
        nh_->logerror("Invalid Servo ID!");
        return;
      }
#if !KONDO
    if (servo_handler_.currentManaged(index)) continue;
#endif
    ServoData& s = servo_handler_.getServo()[index];
    s.torque_enable_ = (control_msg.torque_enable[i] != 0) ? true : false;
    servo_handler_.setTorqueFromPresetnPos(index);

  }
}

void DirectServo::servoConfigCallback(const spinal::SetDirectServoConfig::Request& req, spinal::SetDirectServoConfig::Response& res)
{
  //TODO: using boardConfigCallback in spine.cpp
  uint8_t command = req.command;

  /* special case : data[0] is flag value */
  if(command == spinal::SetDirectServoConfig::Request::SET_DYNAMIXEL_TTL_RS485_MIXED)
    {
      // servo_handler_.setTTLRS485Mixed(req.data[0]);
      FlashMemory::erase();
      FlashMemory::write();
      res.success = true;
      return;
    }

  uint8_t servo_index = req.data[0];
  if(servo_index >= servo_handler_.getServoNum())
    {
      nh_->logerror("Invalid Servo ID!");
      return;
    }
#if !KONDO
  if (servo_handler_.currentManaged(servo_index)) {res.success=false; return;}
#endif
  ServoData& s = servo_handler_.getServo()[servo_index];
  switch (command) {
  case spinal::SetDirectServoConfig::Request::SET_SERVO_HOMING_OFFSET:
    {
      if(!s.torque_enable_){
        int32_t calib_value = req.data[1];
        s.calib_value_ = calib_value;
        servo_handler_.setHomingOffset(servo_index);
        res.success = true;
      }else{
        nh_->logerror("Cannot set homing offset during torque on state.");
        res.success = false;
      }
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_PID_GAIN:
    {
      if(!s.torque_enable_){
        s.p_gain_ = req.data[1];
        s.i_gain_ = req.data[2];
        s.d_gain_ = req.data[3];
        servo_handler_.setPositionGains(servo_index);
        FlashMemory::erase();
        FlashMemory::write();
        res.success = true;
      }else{
        nh_->logerror("Cannot set PID gains during torque on state.");
        res.success = false;
      }
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_PROFILE_VEL:
    {
      s.profile_velocity_ = req.data[1];
      servo_handler_.setProfileVelocity(servo_index);
      FlashMemory::erase();
      FlashMemory::write();
      res.success = true;
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_SEND_DATA_FLAG:
    {
      s.send_data_flag_ = req.data[1];
      FlashMemory::erase();
      FlashMemory::write();
      res.success = true;
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_CURRENT_LIMIT:
    {
      s.current_limit_ = req.data[1];
      servo_handler_.setCurrentLimit(servo_index);
      res.success = true;
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_EXTERNAL_ENCODER_FLAG:
    {
      if(!s.torque_enable_){
        s.external_encoder_flag_ = req.data[1];
        s.first_get_pos_flag_ = true;
        if(!s.external_encoder_flag_)
          { // if use the servo internal encoder, we directly output the encoder value without scaling by resolution_ratio.
            s.servo_resolution_ = 1;
            s.joint_resolution_ = 1;
            s.resolution_ratio_ = 1;
          }
        FlashMemory::erase();
        FlashMemory::write();
        res.success = true;
      }else{
        nh_->logerror("Cannot set ex encoder falg during torque on state.");
        res.success = false;
      }
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_RESOLUTION_RATIO:
    {
      if(!s.torque_enable_){
        s.joint_resolution_ = req.data[1];
        s.servo_resolution_ = req.data[2];
        s.hardware_error_status_ &= ((1 << RESOLUTION_RATIO_ERROR) - 1); // 0b00111111: reset

        if(s.servo_resolution_ == 65535 || s.joint_resolution_ == 65535){
          s.hardware_error_status_ |= (1 << RESOLUTION_RATIO_ERROR);  // 0b01000000;
          s.resolution_ratio_ = 1;
        }
        else{
          s.resolution_ratio_ = (float)s.servo_resolution_ / (float)s.joint_resolution_;
          s.first_get_pos_flag_ = true;
          FlashMemory::erase();
          FlashMemory::write();
        }
        res.success = true;
      }else{
        nh_->logerror("Cannot set resolution rate during torque on state.");
        res.success = false;
      }
      
      break;
    }
  default:
    break;
  }
  // res.success = true;
}

void DirectServo::boardInfoCallback(const spinal::GetBoardInfo::Request& req, spinal::GetBoardInfo::Response& res)
{
  //TODO: Bad implementation. This features should not be located in servo interface.
  spinal::BoardInfo& board = board_info_res_.boards[0];
  board.imu_send_data_flag = 1;
#if DYNAMIXEL
  board.dynamixel_ttl_rs485_mixed = servo_handler_.getTTLRS485Mixed();
#endif
  board.slave_id = 0;
  for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
    const ServoData& s = servo_handler_.getServo()[i];
    board.servos[i].id = s.id_;
    board.servos[i].p_gain = s.p_gain_;
    board.servos[i].i_gain = s.i_gain_;
    board.servos[i].d_gain = s.d_gain_;
    board.servos[i].profile_velocity = s.profile_velocity_;
    board.servos[i].current_limit = s.current_limit_;
    board.servos[i].send_data_flag = s.send_data_flag_;
    board.servos[i].external_encoder_flag = s.external_encoder_flag_;
    board.servos[i].joint_resolution = s.joint_resolution_;
    board.servos[i].servo_resolution = s.servo_resolution_;
  }
  res = board_info_res_;
}

void DirectServo::jointProfilesCallback(const spinal::JointProfiles& joint_prof_msg)
{
  for(unsigned int i = 0; i  < joint_prof_msg.joints_length; i++){
    joint_profiles_[i].servo_id = joint_prof_msg.joints[i].servo_id;
    joint_profiles_[i].angle_sgn = joint_prof_msg.joints[i].angle_sgn;
    joint_profiles_[i].angle_scale = joint_prof_msg.joints[i].angle_scale;
    joint_profiles_[i].zero_point_offset = joint_prof_msg.joints[i].zero_point_offset;
  }
}

void DirectServo::directCurrentCallback(const spinal::DirectServoCurrent& msg)
{
#if !KONDO
  if (msg.index_length!=msg.session_length || msg.index_length!=msg.sequence_length ||
      msg.index_length!=msg.current_length || msg.index_length>MAX_SERVO_NUM) return;
  // Validate the entire packet before queueing any joint.
  for (unsigned i=0; i<msg.index_length; ++i) {
    if (msg.index[i]>=servo_handler_.getServoNum()) return;
    for (unsigned j=0; j<i; ++j) if (msg.index[i]==msg.index[j]) return;
  }
  for (unsigned i=0; i<msg.index_length; ++i)
    servo_handler_.currentCommand(msg.index[i],msg.session[i],msg.sequence[i],msg.current[i]);
#endif
}

void DirectServo::directModeCallback(const spinal::DirectServoMode::Request& req,spinal::DirectServoMode::Response& res)
{
  res.success=false;
  res.status=spinal_current::DENIED;
#if !KONDO
  DynamixelSerial::CurrentResult result;
  servo_handler_.currentRequest(req.index,req.servo_id,req.action,req.session,req.current_limit,result);
  res.success=result.success; res.status=result.status;
  res.servo_id=(req.index<servo_handler_.getServoNum()) ? servo_handler_.getServo()[req.index].id_ : 255;
  res.model_number=result.snapshot.model; res.firmware_version=result.snapshot.firmware;
  res.operating_mode=result.snapshot.mode; res.current_limit=result.snapshot.limit;
  res.torque_enabled=result.snapshot.torque; res.position=result.snapshot.position;
  res.session=result.token; res.timeout_ms=spinal_current::Session::TIMEOUT_MS;
#endif
}
