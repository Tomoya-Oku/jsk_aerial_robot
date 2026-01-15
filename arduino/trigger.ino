#include <ros.h>
#include <std_msgs/UInt8.h>
constexpr uint8_t BUTTON_PIN = 2;
constexpr unsigned long DEBOUNCE_MS = 15;

ros::NodeHandle nh;
std_msgs::UInt8 msg;
ros::Publisher pub("/twin_hammer/button", &msg);
uint8_t last_reading = HIGH;   // INPUT_PULLUPなので未押下=HIGH
uint8_t stable_state = HIGH;
unsigned long last_toggle_ms = 0;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(57600);
  nh.getHardware()->setBaud(57600);  // PC側と合わせる
  nh.initNode();
  nh.advertise(pub);
}

void loop() {
  uint8_t reading = digitalRead(BUTTON_PIN);
  unsigned long now = millis();
  
  if (reading != last_reading) {      // 変化検出
    last_toggle_ms = now;
    last_reading = reading;
  }
  // 一定時間同じなら安定とみなして送信
  if ((now - last_toggle_ms) > DEBOUNCE_MS && reading != stable_state) {
    stable_state = reading;           // HIGH=離す, LOW=押す
    msg.data = (stable_state == LOW) ? 1 : 0;
    Serial.println((int)msg.data);
    pub.publish(&msg);
  }
  nh.spinOnce();
  delay(5);
}
