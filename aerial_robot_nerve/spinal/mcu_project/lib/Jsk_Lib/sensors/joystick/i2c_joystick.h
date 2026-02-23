#pragma once

#include <cstdint>

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

#include "main.h"  // I2C_HandleTypeDef, HAL_I2C_*

// ==============================
// M5Stack JoyStick2 (STM32) I2C Protocol
// - Default I2C address: 0x63
// - X 16bit: 0x00(L), 0x01(H)
// - Y 16bit: 0x02(L), 0x03(H)
// - Button:  0x20 (1:not press, 0:press)
// Ref: M5Stack Unit JoyStick2 I2C Protocol
// ==============================

#ifndef M5JS2_I2C_ADDR_7BIT
#define M5JS2_I2C_ADDR_7BIT 0x63
#endif

#ifndef M5JS2_TOPIC
#define M5JS2_TOPIC "joy_i2c"
#endif

#ifndef M5JS2_I2C_TIMEOUT_MS
// core loopを止めないために短め推奨
#define M5JS2_I2C_TIMEOUT_MS 3
#endif

class I2CJoystick
{
public:
  I2CJoystick() : pub_(M5JS2_TOPIC, &msg_)
  {
  }

  void init(I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh)
  {
    hi2c_ = hi2c;
    nh_ = nh;

    msg_.data = data_;
    msg_.data_length = 3;  // [x, y, btn]

    nh_->advertise(pub_);
  }

  // 失敗時は即return（制御ループを止めない）
  void update()
  {
    if (!hi2c_ || !nh_)
      return;

    // X/Y 16bit を一括読み（0x00〜0x03）
    uint8_t xy[4] = { 0 };

    const uint16_t addr8 = static_cast<uint16_t>(M5JS2_I2C_ADDR_7BIT) << 1;

    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c_, addr8,
                                            0x00,  // Joy1 X ADC Value-L
                                            I2C_MEMADD_SIZE_8BIT, xy, sizeof(xy), M5JS2_I2C_TIMEOUT_MS);

    if (st != HAL_OK)
      return;

    // little-endian: ADC = L + H*256 (0..65535)
    const uint16_t x_u16 = static_cast<uint16_t>(xy[0]) | (static_cast<uint16_t>(xy[1]) << 8);
    const uint16_t y_u16 = static_cast<uint16_t>(xy[2]) | (static_cast<uint16_t>(xy[3]) << 8);

    // Button (0x20): 1:not press, 0:press
    uint8_t b = 0xFF;
    st = HAL_I2C_Mem_Read(hi2c_, addr8, 0x20, I2C_MEMADD_SIZE_8BIT, &b, 1, M5JS2_I2C_TIMEOUT_MS);

    if (st != HAL_OK)
      return;

    // rostopic echoで見やすいように int16 に落とす（0..65535 は負になる領域あり）
    // もし 0..65535 を保持したいなら UInt16MultiArray 等に変えてください。
    data_[0] = static_cast<int16_t>(x_u16);
    data_[1] = static_cast<int16_t>(y_u16);
    data_[2] = static_cast<int16_t>(b & 0x01);

    pub_.publish(&msg_);
  }

  // 任意: RGB LED (0x30..0x32) に書く（必要なら使う）
  // void setRgb(uint8_t r, uint8_t g, uint8_t b);

private:
  I2C_HandleTypeDef* hi2c_{ nullptr };
  ros::NodeHandle* nh_{ nullptr };

  std_msgs::Int16MultiArray msg_;
  int16_t data_[3] = { 0, 0, 1 };
  ros::Publisher pub_;
};