
#ifdef __cplusplus
#include <std_msgs/UInt16MultiArray.h>
#endif

#ifdef __cplusplus

namespace joystick
{

// ====== 内部状態（C++11なのでここに実体を置く） ======
static ros::NodeHandle* nh_ = NULL;
static ADC_HandleTypeDef* hadc_ = NULL;

static uint32_t ch_x_ = ADC_CHANNEL_18;
static uint32_t ch_y_ = ADC_CHANNEL_19;
static uint32_t sampling_time_ = ADC_SAMPLETIME_64CYCLES_5;
static uint32_t period_ms_ = 20;

static volatile bool ready_ = false;

static std_msgs::UInt16MultiArray msg_;
static uint16_t buf_[2] = { 0, 0 };
static ros::Publisher pub_("joystick_adc", &msg_);

// ====== Battery側に戻す（CH15に戻す）を joystick 内に吸収 ======
// ここは「あなたの元の実装」に合わせて調整が必要。
// まずは一般形で CH15/Rank1/SamplingTime を設定する関数を用意。
static void RestoreBatteryAdcChannel15_()
{
  if (!hadc_)
    return;

  ADC_ChannelConfTypeDef sConfig = { 0 };
  HAL_ADC_Stop(hadc_);

  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;

  // battery_status 側の想定 sampling に合わせたいならここを適切な値へ
  // 例: ADC_SAMPLETIME_64CYCLES_5 / 810.5 / etc
  sConfig.SamplingTime = sampling_time_;

#if defined(ADC_SINGLE_ENDED)
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
#if defined(DISABLE)
  sConfig.OffsetSignedSaturation = DISABLE;
#endif
#endif

  (void)HAL_ADC_ConfigChannel(hadc_, &sConfig);
}

// ====== ADC 1回読み ======
static uint16_t ReadAdcOnce(uint32_t channel)
{
  if (!hadc_)
    return 0;

  ADC_ChannelConfTypeDef sConfig = { 0 };

  HAL_ADC_Stop(hadc_);

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = sampling_time_;

#if defined(ADC_SINGLE_ENDED)
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
#if defined(DISABLE)
  sConfig.OffsetSignedSaturation = DISABLE;
#endif
#endif

  if (HAL_ADC_ConfigChannel(hadc_, &sConfig) != HAL_OK)
    return 0;
  if (HAL_ADC_Start(hadc_) != HAL_OK)
    return 0;
  if (HAL_ADC_PollForConversion(hadc_, 10) != HAL_OK)
    return 0;

  uint16_t v = (uint16_t)HAL_ADC_GetValue(hadc_);
  HAL_ADC_Stop(hadc_);
  return v;
}

// ====== 公開API ======
void setup(ros::NodeHandle* nh, ADC_HandleTypeDef* hadc, uint32_t adc_ch_x, uint32_t adc_ch_y, uint32_t samp_time,
           uint32_t pub_period_ms)
{
  nh_ = nh;
  hadc_ = hadc;
  ch_x_ = adc_ch_x;
  ch_y_ = adc_ch_y;
  sampling_time_ = samp_time;
  period_ms_ = pub_period_ms;

  msg_.data_length = 2;
  msg_.data = buf_;

  if (nh_)
  {
    nh_->advertise(pub_);
    ready_ = true;
  }
}

void task(void const* /*argument*/)
{
  while (!ready_)
  {
    osDelay(1);
  }

  for (;;)
  {
    buf_[0] = ReadAdcOnce(ch_x_);
    buf_[1] = ReadAdcOnce(ch_y_);

    // battery 側が ADC CH15 を使う想定なら戻す
    RestoreBatteryAdcChannel15_();

    pub_.publish(&msg_);
    osDelay(period_ms_);
  }
}

}  // namespace joystick
#endif
