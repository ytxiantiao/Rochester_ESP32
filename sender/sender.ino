#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <VL53L0X.h>


uint8_t RECEIVER_MAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};   //替换：接收端 ESP32-S3 的真实 MAC 地址

VL53L0X sensor;

typedef struct {
  uint16_t distance_mm;
} SensorPacket;

SensorPacket packet;
unsigned long lastSendMs = 0;
const unsigned long SEND_INTERVAL_MS = 33;

uint16_t lastValidDistance = 1000; 
int failCount = 0;
int retryCount = 0;
unsigned long lastRetryTime = 0;

void initSensor() {
  sensor.setTimeout(500);
  if (sensor.init()) {
    sensor.setMeasurementTimingBudget(33000);
    sensor.startContinuous();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);
  Wire.setClock(400000);

  initSensor(); 

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) return;

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  unsigned long now = millis();
  
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;
    
    uint16_t distance = sensor.readRangeContinuousMillimeters();
    
    //异常值
    if (distance >= 8000) { 
      // 超过量程
      distance = 2000;
      failCount++; 
    } 
    else if (distance == 0) {
      // 盲区保持原值
      distance = lastValidDistance; 
      failCount++;
    } 
    else {
      // 正常
      lastValidDistance = distance;
      failCount = 0; 
      retryCount = 0; 
    }

    //看门狗
    if (failCount > 30) {
      // 如果距离上次重试超过 1.5 秒
      if (now - lastRetryTime > 1500) {
        lastRetryTime = now;
        Serial.println("尝试复位 I2C 总线与传感器...");
        
        Wire.end();
        delay(50);
        Wire.begin(8, 9);
        Wire.setClock(400000);
        
        sensor.setTimeout(500);
        if (sensor.init()) {
          sensor.setMeasurementTimingBudget(33000);
          sensor.startContinuous();
          Serial.println("复位成功！");
          failCount = 0;
        } else {
          Serial.println("复位失败！");
          retryCount++;
        }

        // 如果连续尝试 5 次（约 7.5 秒）依然失败，直接硬重启整个 ESP32 都怪你
        if (retryCount >= 5) {
          Serial.println("传感器彻底死机，触发系统硬重启！");
          ESP.restart(); 
        }
      }
    }
    
    packet.distance_mm = distance;
    esp_now_send(RECEIVER_MAC, (uint8_t *)&packet, sizeof(packet));
  }
}