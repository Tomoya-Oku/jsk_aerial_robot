#pragma once

#include <cstdint>

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

#include "main.h"  // I2C_HandleTypeDef, HAL_I2C_*

#ifndef M5JS2_I2C_ADDR_7BIT
#define M5JS2_I2C_ADDR_7BIT 0x63
#endif

#ifndef M5JS2_TOPIC
#define M5JS2_TOPIC "joy_i2c"
#endif

#ifndef M5JS2_I2C_TIMEOUT_MS
#define M5JS2_I2C_TIMEOUT_MS 3
#endif

#ifndef M5JS2_CALIB_SAMPLES
// 起動時にこの回数だけサンプルして平均をオフセットにする
#define M5JS2_CALIB_SAMPLES 16
#endif

#ifndef M5JS2_CALIB_DELAY_MS
// サンプル間隔（coreTaskがRTOS前にinit呼ぶ前提ならHAL_Delayが使える）
#define M5JS2_CALIB_DELAY_MS 2
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
    msg_.data_length = 3;  // [dx, dy, btn]
    nh_->advertise(pub_);

    // ここで初期キャリブ（中心を0に）
    calibrate();
  }

  // 任意：後からキャリブしたい場合に呼ぶ
  void calibrate()
  {
    if (!hi2c_)
      return;

    uint32_t sum_x = 0;
    uint32_t sum_y = 0;
    uint16_t last_x = 0, last_y = 0;

    uint16_t x = 0, y = 0;
    bool ok_cnt = 0;

    for (int i = 0; i < M5JS2_CALIB_SAMPLES; ++i)
    {
      if (readXY(x, y))
      {
        sum_x += x;
        sum_y += y;
        last_x = x;
        last_y = y;
        ok_cnt = true;
      }
#if M5JS2_CALIB_DELAY_MS > 0
      HAL_Delay(M5JS2_CALIB_DELAY_MS);
#endif
    }

    if (ok_cnt)
    {
      x0_ = static_cast<uint16_t>(sum_x / M5JS2_CALIB_SAMPLES);
      y0_ = static_cast<uint16_t>(sum_y / M5JS2_CALIB_SAMPLES);
      calibrated_ = true;
    }
    else
    {
      // 読めなかったら、とりあえず最後値（0のまま）扱い
      x0_ = last_x;
      y0_ = last_y;
      calibrated_ = false;
    }
  }

  void update()
  {
    if (!hi2c_ || !nh_)
      return;

    uint16_t x = 0, y = 0;
    if (!readXY(x, y))
      return;

    uint8_t b = 0xFF;
    if (!readButton(b))
      return;

    // dx,dy: 初期値を引いて0中心に
    int32_t dx = static_cast<int32_t>(x) - static_cast<int32_t>(x0_);
    int32_t dy = static_cast<int32_t>(y) - static_cast<int32_t>(y0_);

    // int16に収める（万一のオーバーフロー対策）
    data_[0] = clampToI16(dx);
    data_[1] = clampToI16(dy);

    // 1:not press, 0:press
    data_[2] = static_cast<int16_t>(b & 0x01);

    pub_.publish(&msg_);
  }

private:
  static int16_t clampToI16(int32_t v)
  {
    if (v > 32767)
      return 32767;
    if (v < -32768)
      return -32768;
    return static_cast<int16_t>(v);
  }

  bool readXY(uint16_t& x, uint16_t& y)
  {
    uint8_t xy[4] = { 0 };
    const uint16_t addr8 = static_cast<uint16_t>(M5JS2_I2C_ADDR_7BIT) << 1;

    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c_, addr8,
                                            0x00,  // X_L
                                            I2C_MEMADD_SIZE_8BIT, xy, sizeof(xy), M5JS2_I2C_TIMEOUT_MS);

    if (st != HAL_OK)
      return false;

    x = static_cast<uint16_t>(xy[0]) | (static_cast<uint16_t>(xy[1]) << 8);
    y = static_cast<uint16_t>(xy[2]) | (static_cast<uint16_t>(xy[3]) << 8);
    return true;
  }

  bool readButton(uint8_t& b)
  {
    const uint16_t addr8 = static_cast<uint16_t>(M5JS2_I2C_ADDR_7BIT) << 1;

    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c_, addr8,
                                            0x20,  // Button
                                            I2C_MEMADD_SIZE_8BIT, &b, 1, M5JS2_I2C_TIMEOUT_MS);

    return (st == HAL_OK);
  }

private:
  I2C_HandleTypeDef* hi2c_{ nullptr };
  ros::NodeHandle* nh_{ nullptr };

  std_msgs::Int16MultiArray msg_;
  int16_t data_[3] = { 0, 0, 1 };
  ros::Publisher pub_;

  // キャリブ用オフセット（起動時のX,Y）
  uint16_t x0_{ 0 }, y0_{ 0 };
  bool calibrated_{ false };
};