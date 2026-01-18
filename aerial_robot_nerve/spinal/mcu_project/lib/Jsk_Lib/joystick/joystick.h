#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os.h"
#include "main.h"  // ADC_HandleTypeDef, ADC_CHANNEL_x, ADC_SAMPLETIME_x など
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

/**
 * Joystick module (header-only)
 * - ADC CHx, CHy を ReadAdcOnce で読む
 * - Battery 用 ADC CH15 に戻すためのコールバックを呼ぶ（任意）
 * - rosserial publish を周期実行する task を提供
 */
namespace joystick
{
// ----- 内部状態（ヘッダオンリーなので inline/static で ODR 回避） -----
inline ros::NodeHandle* nh = nullptr;
inline ADC_HandleTypeDef* hadc = nullptr;

inline uint32_t ch_x = ADC_CHANNEL_18;
inline uint32_t ch_y = ADC_CHANNEL_19;
inline uint32_t sampling_time = ADC_SAMPLETIME_64CYCLES_5;

inline uint32_t period_ms = 20;                  // publish周期（20ms=50Hz）
inline void (*restore_battery_adc)() = nullptr;  // CH15へ戻す用（任意）

inline volatile bool ready = false;

inline std_msgs::UInt16MultiArray msg;
inline uint16_t buf[2] = { 0, 0 };

// Topic は固定（Publisher ctor が topic 文字列を必要とするため）
inline ros::Publisher pub("joystick_adc", &msg);

// ----- 内部ユーティリティ -----
inline uint16_t ReadAdcOnce(uint32_t channel, uint32_t samp_time)
{
  if (!hadc)
    return 0;

  ADC_ChannelConfTypeDef sConfig = { 0 };

  // 念のため停止（すでに動いていてもOK）
  HAL_ADC_Stop(hadc);

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = samp_time;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;

  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
    return 0;
  if (HAL_ADC_Start(hadc) != HAL_OK)
    return 0;
  if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK)
    return 0;

  uint16_t v = (uint16_t)HAL_ADC_GetValue(hadc);
  HAL_ADC_Stop(hadc);
  return v;
}

// ----- 公開API -----

/**
 * setup: coreTask内で nh_.initNode() の後に1回だけ呼ぶ
 * @param nh_in ros::NodeHandle*
 * @param hadc_in ADC_HandleTypeDef*（例: &hadc1）
 * @param restore_cb BatteryStatus側が前提にしている ADC設定へ戻す関数（任意）
 * @param adc_ch_x joystick X の ADC チャンネル（デフォルト CH18）
 * @param adc_ch_y joystick Y の ADC チャンネル（デフォルト CH19）
 * @param samp_time ADC sampling time（デフォルト 64cycles）
 * @param pub_period_ms publish周期（デフォルト 20ms）
 */
inline void setup(ros::NodeHandle* nh_in, ADC_HandleTypeDef* hadc_in, void (*restore_cb)() = nullptr,
                  uint32_t adc_ch_x = ADC_CHANNEL_18, uint32_t adc_ch_y = ADC_CHANNEL_19,
                  uint32_t samp_time = ADC_SAMPLETIME_64CYCLES_5, uint32_t pub_period_ms = 20)
{
  nh = nh_in;
  hadc = hadc_in;
  restore_battery_adc = restore_cb;
  ch_x = adc_ch_x;
  ch_y = adc_ch_y;
  sampling_time = samp_time;
  period_ms = pub_period_ms;

  msg.data_length = 2;
  msg.data = buf;

  // advertise は initNode 後に呼ぶ必要がある
  if (nh)
  {
    nh->advertise(pub);
    ready = true;
  }
}

/**
 * FreeRTOS/CMSIS-OS task entry
 * osThreadDef(..., joystick::task, ...)
 */
inline void task(void const* /*argument*/)
{
  // setup 完了待ち
  while (!ready)
  {
    osDelay(1);
  }

  for (;;)
  {
    buf[0] = ReadAdcOnce(ch_x, sampling_time);
    buf[1] = ReadAdcOnce(ch_y, sampling_time);

    // Battery 側のADC前提に戻す（必要なら）
    if (restore_battery_adc)
      restore_battery_adc();

    pub.publish(&msg);

    // 送信は別スレッド rosPublishTask が捌く設計なので、ここで spinOnce は基本不要
    // （必要なら呼んでもいいが、publish専用なら呼ばない方がスッキリ）
    osDelay(period_ms);
  }
}

}  // namespace joystick

#endif  // __cplusplus