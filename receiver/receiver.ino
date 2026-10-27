#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <U8g2lib.h>
#include <Wire.h>

#define BOOT_PIN 0
unsigned long lastBtnMs = 0;
int currentMode = 0;
const int TOTAL_MODES = 10; 
bool serialTriggered = false;
unsigned long serialTriggerTime = 0;
unsigned long flashHoldTime = 0; 

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(
    U8G2_R0, U8X8_PIN_NONE, /* clock=*/ 9, /* data=*/ 8
);

#define HISTORY_LEN 88     
uint16_t distRing[HISTORY_LEN];  
float speedFiltRing[HISTORY_LEN]; 
float speedRawRing[HISTORY_LEN];  
uint8_t head = 0;

#define PLOT_TOP 12        
#define PLOT_BOTTOM 63
#define PLOT_HEIGHT (PLOT_BOTTOM - PLOT_TOP) 
#define PLOT_LEFT 20

#define Y_HIGH_PCT 95
#define Y_LOW_PCT  5

#define MIN_SPAN_MM 200
float dispMin = 800.0f;
float dispMax = 1200.0f;
float dispSpeedMax = 300.0f; 

uint16_t latestDistance = 1000;
float latestSpeedFilt = 0;
float latestSpeedRaw = 0;
bool hasNewDistance = false;
unsigned long lastDrawMs = 0;
unsigned long lastRecvMs = 0; 
const unsigned long DRAW_INTERVAL_MS = 30;

// 滤波窗口为5
#define DIFF_WINDOW_SIZE 5
float rawDiffBuffer[DIFF_WINDOW_SIZE] = {0};
uint8_t diffIndex = 0;
float weights[DIFF_WINDOW_SIZE];
float weightSum = 0;

unsigned long lastRecvTime = 0;
uint16_t lastRecvDist = 0;

typedef struct {
  uint16_t distance_mm;
} SensorPacket;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(SensorPacket)) return;
  SensorPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.distance_mm == 0) return;
  
  unsigned long now = millis();
  lastRecvMs = now; 
  
  if (lastRecvTime != 0) {
    float dt = (now - lastRecvTime) / 1000.0f;
    if (dt > 0.001f) {
      latestSpeedRaw = (pkt.distance_mm - lastRecvDist) / dt;
      if (latestSpeedRaw > 8000) latestSpeedRaw = 8000;
      if (latestSpeedRaw < -8000) latestSpeedRaw = -8000;

      rawDiffBuffer[diffIndex] = latestSpeedRaw;
      diffIndex = (diffIndex + 1) % DIFF_WINDOW_SIZE;

      float sum = 0;
      for (int i = 0; i < DIFF_WINDOW_SIZE; i++) {
        int idx = (diffIndex + i) % DIFF_WINDOW_SIZE; 
        sum += rawDiffBuffer[idx] * weights[i]; 
      }
      latestSpeedFilt = sum / weightSum;
    }
  }
  
  lastRecvTime = now;
  lastRecvDist = pkt.distance_mm;
  latestDistance = pkt.distance_mm;
  hasNewDistance = true;
}

void setup() {
  Serial.begin(115200);
  lastRecvMs = millis();
  pinMode(BOOT_PIN, INPUT_PULLUP);
  
  weightSum = 0;
  for (int i = 0; i < DIFF_WINDOW_SIZE; i++) {
    weights[i] = pow(0.85, DIFF_WINDOW_SIZE - 1 - i); 
    weightSum += weights[i];
  }
  
  for (int i = 0; i < HISTORY_LEN; i++) {
    distRing[i] = 1000;
    speedFiltRing[i] = 0;
    speedRawRing[i] = 0;
  }

  Wire.begin(8, 9);
  Wire.setClock(400000);
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); 

  if (esp_now_init() != ESP_OK) return;
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (digitalRead(BOOT_PIN) == LOW) {
    if (millis() - lastBtnMs > 250) {
      lastBtnMs = millis();
      currentMode = (currentMode + 1) % TOTAL_MODES;
    }
  }

  unsigned long now = millis();
  
  // 突变flash全局判定
  uint16_t oldDist = distRing[(head - 10 + HISTORY_LEN) % HISTORY_LEN];
  int longDelta = abs(latestDistance - oldDist); 
  float shortDelta = abs(latestSpeedFilt);       
  bool isTrigger = (longDelta > 250 && shortDelta > 500);

  // 全局串口触发 (2秒)
  if (isTrigger && !serialTriggered) {
    Serial.println("start");
    serialTriggered = true;
    serialTriggerTime = now;
  }
  if (serialTriggered && (now - serialTriggerTime > 2000)) {
    Serial.println("stop");
    serialTriggered = false;
  }

  // UI反色 (5秒)
  if (isTrigger) {
    flashHoldTime = now;
  }

  if (now - lastDrawMs >= DRAW_INTERVAL_MS) {
    lastDrawMs = now;
    
    if (now - lastRecvMs > 500) hasNewDistance = true; 

    if (hasNewDistance) {
      distRing[head] = latestDistance;
      speedFiltRing[head] = latestSpeedFilt;
      speedRawRing[head] = latestSpeedRaw;
      head = (head + 1) % HISTORY_LEN;
      hasNewDistance = false;
    }
    drawScreen();
  }
}

float getNiceStep(float span) {
  if (span <= 0) return 10.0f;
  int targetDivisions = 5; 
  float rawStep = span / targetDivisions;
  float mag = pow(10, floor(log10(rawStep)));
  float norm = rawStep / mag;
  if (norm < 1.5) return mag;
  if (norm < 3.5) return 2 * mag;
  if (norm < 7.5) return 5 * mag;
  return 10 * mag;
}

void drawScale(float actualMin, float actualMax, int yHigh, int yLow, bool isLeft) {
  float span = actualMax - actualMin;
  if (span <= 0) return;
  float step = getNiceStep(span);
  float startVal = ceil(actualMin / step) * step;
  u8g2.setFont(u8g2_font_4x6_tf);
  
  for (float val = startVal; val <= actualMax; val += step) {
    float t = (val - actualMin) / span;
    int y = (uint8_t)(yLow - t * (yLow - yHigh));
    char buf[8];
    snprintf(buf, sizeof(buf), "%d", (int)val);
    
    if (isLeft) {
      int w = u8g2.getStrWidth(buf);
      int textX = PLOT_LEFT - 3 - w;
      if (textX < 0) textX = 0;
      u8g2.setCursor(textX, y + 2);
      u8g2.print(buf);
      u8g2.drawLine(PLOT_LEFT - 2, y, PLOT_LEFT - 1, y);
    } else {
      u8g2.setCursor(128 - 2 - u8g2.getStrWidth(buf), y + 2);
      u8g2.print(buf);
      u8g2.drawLine(126, y, 127, y);
    }
  }
}

void drawScreen() {
  if (millis() - lastRecvMs > 1500) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB14_tr); 
    u8g2.drawStr(25, 35, "ERROR");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(15, 55, "NO SIGNAL");
    u8g2.sendBuffer();
    return;
  }

  uint16_t wMin = 65535, wMax = 0;
  float sMax = 0;
  
  // 模式 1,4,6属于 Raw 模式
  bool isRawMode = (currentMode == 1 || currentMode == 4 || currentMode == 6); 
  
  for (int i = 0; i < HISTORY_LEN; i++) {
    if (distRing[i] < wMin) wMin = distRing[i];
    if (distRing[i] > wMax) wMax = distRing[i];
    
    float currentS = isRawMode ? speedRawRing[i] : speedFiltRing[i];
    float absS = abs(currentS);
    if (absS > sMax) sMax = absS;
  }

  float tMin = (float)wMin;
  float tMax = (float)wMax;
  if (tMax > dispMax) dispMax = tMax;
  else dispMax = dispMax * 0.85f + tMax * 0.15f;
  if (tMin < dispMin) dispMin = tMin;
  else dispMin = dispMin * 0.85f + tMin * 0.15f;
  
  float span = dispMax - dispMin;
  if (span < (float)MIN_SPAN_MM) {
    float mid = (dispMax + dispMin) * 0.5f;
    dispMax = mid + MIN_SPAN_MM * 0.5f;
    dispMin = mid - MIN_SPAN_MM * 0.5f;
    if (dispMin < 0) { dispMin = 0; dispMax = (float)MIN_SPAN_MM; }
  }

  if (sMax > dispSpeedMax) dispSpeedMax = sMax;
  else dispSpeedMax = dispSpeedMax * 0.85f + sMax * 0.15f;
  if (dispSpeedMax < 100.0f) dispSpeedMax = 100.0f; 

  float actualSpeedMin = -dispSpeedMax;
  float actualSpeedMax = dispSpeedMax;

  int yHigh = PLOT_BOTTOM - (int)(PLOT_HEIGHT * Y_HIGH_PCT / 100.0f);
  int yLow  = PLOT_BOTTOM - (int)(PLOT_HEIGHT * Y_LOW_PCT  / 100.0f);

  u8g2.clearBuffer();
  u8g2.setDrawColor(1);

  // 7模式Flash 突变反色
  if (currentMode == 7) {
    bool isUIFlash = (millis() - flashHoldTime < 5000);

    if (isUIFlash) {
      u8g2.setDrawColor(1);
      u8g2.drawBox(0, 0, 128, 64); 
      u8g2.setDrawColor(0);        
    }

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(0, 9);
    u8g2.print("D:");
    u8g2.print(latestDistance);


    u8g2.print(" V:");
    // 都怪你
    u8g2.printf("%4d", (int)latestSpeedFilt);





    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.setCursor(95, 7);
    u8g2.print("Flash");

    drawScale(dispMin, dispMax, yHigh, yLow, true); 
    drawScale(actualSpeedMin, actualSpeedMax, yHigh, yLow, false);

    int plotRightBound = 104; 
    int plotWidth = plotRightBound - PLOT_LEFT;

    for (int i = 0; i < HISTORY_LEN; i++) {
      uint8_t idx = (head + i) % HISTORY_LEN;
      int x = PLOT_LEFT + (int)((float)i / HISTORY_LEN * plotWidth);
      
      float tD = ((float)distRing[idx] - dispMin) / (dispMax - dispMin);
      if (tD < 0) tD = 0; if (tD > 1) tD = 1;
      int yD = (uint8_t)(yLow - tD * (yLow - yHigh));

      float tS = (speedFiltRing[idx] - actualSpeedMin) / (actualSpeedMax - actualSpeedMin);
      if (tS < 0) tS = 0; if (tS > 1) tS = 1;
      int yS = (uint8_t)(yLow - tS * (yLow - yHigh));

      if (i < HISTORY_LEN - 1) {
        uint8_t nextIdx = (head + i + 1) % HISTORY_LEN;
        int nextX = PLOT_LEFT + (int)((float)(i + 1) / HISTORY_LEN * plotWidth);
        
        float tD2 = ((float)distRing[nextIdx] - dispMin) / (dispMax - dispMin);
        if (tD2 < 0) tD2 = 0; if (tD2 > 1) tD2 = 1;
        int nyD = (uint8_t)(yLow - tD2 * (yLow - yHigh));
        u8g2.drawLine(x, yD, nextX, nyD);

        float tS2 = (speedFiltRing[nextIdx] - actualSpeedMin) / (actualSpeedMax - actualSpeedMin);
        if (tS2 < 0) tS2 = 0; if (tS2 > 1) tS2 = 1;
        int nyS = (uint8_t)(yLow - tS2 * (yLow - yHigh));
        u8g2.drawLine(x, yS, nextX, nyS);
      }
    }
    u8g2.sendBuffer();
    return;
  }

  // 纯数值模式，雷霆大字
  if (currentMode == 8 || currentMode == 9) {
    u8g2.clearBuffer();
    char buf[16];
    u8g2.setFont(u8g2_font_logisoso24_tf); 
    
    if (currentMode == 8) {
      snprintf(buf, sizeof(buf), "%d", latestDistance);
      int w = u8g2.getStrWidth(buf);
      u8g2.setCursor((128 - w) / 2, 45);
      u8g2.print(buf);
      
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0, 10);
      u8g2.print("Dist (mm)");




      // 语文作业好多
      /*嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      */
    } else {
      snprintf(buf, sizeof(buf), "%d", (int)latestSpeedFilt); 
      int w = u8g2.getStrWidth(buf);
      u8g2.setCursor((128 - w) / 2, 45);
      u8g2.print(buf);
      
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0, 10);
      u8g2.print("Speed Filt (mm/s)");
      
      u8g2.setCursor(0, 63);
      u8g2.print("VR:");
      u8g2.print((int)latestSpeedRaw);
    }
    u8g2.sendBuffer();
    return;
  }

  //  顶部文字 
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(0, 9);
  
  // 再加一个
  if (isRawMode) {
    u8g2.print("D:");
    u8g2.print(latestDistance);
    u8g2.print(" V:");
    u8g2.printf("%4d", (int)latestSpeedFilt);
    u8g2.print(" R:");
    u8g2.printf("%4d", (int)latestSpeedRaw);
  } else {
    u8g2.print("D:");
    u8g2.print(latestDistance);
    u8g2.print(" V:");
    u8g2.printf("%4d", (int)latestSpeedFilt);
  }

  const char* modeNames[] = {
    "D&V F", "D&V R", "D", "V F", "V R", "XY F", "XY R", "Flash", "BigD", "BigVF"
  };
  
  // 1,4,5,6不要标签 好挤
  if (currentMode != 1 && currentMode != 4 && currentMode != 5 && currentMode != 6) {
    u8g2.setFont(u8g2_font_4x6_tf);
    u8g2.setCursor(95, 7);
    u8g2.print(modeNames[currentMode]);
  }

  // xy模式
  if (currentMode == 5 || currentMode == 6) {
    int xyLeft = 24, xyRight = 124, xyTop = 12, xyBottom = 60;
    u8g2.drawFrame(xyLeft, xyTop, xyRight - xyLeft, xyBottom - xyTop);

    float stepD = getNiceStep(dispMax - dispMin);
    float startD = ceil(dispMin / stepD) * stepD;
    for (float val = startD; val <= dispMax; val += stepD) {
      int x = xyLeft + (int)(((val - dispMin) / (dispMax - dispMin)) * (xyRight - xyLeft));
      if (x >= xyLeft && x <= xyRight) {
        u8g2.drawLine(x, xyBottom, x, xyBottom + 2);
        char buf[8]; snprintf(buf, sizeof(buf), "%d", (int)val);

        int w = u8g2.getStrWidth(buf);
        int textX = x - w/2; 
        if (textX < 0) textX = 0;


        if (textX + w > 128) textX = 128 - w;

        u8g2.setCursor(textX, 63);
        u8g2.print(buf);
      }
    }

    float stepS = getNiceStep(dispSpeedMax * 2);
    float startS = ceil(actualSpeedMin / stepS) * stepS;
    for (float val = startS; val <= actualSpeedMax; val += stepS) {
      int y = xyBottom - (int)(((val - actualSpeedMin) / (actualSpeedMax - actualSpeedMin)) * (xyBottom - xyTop));
      if (y >= xyTop && y <= xyBottom) {
        u8g2.drawLine(xyLeft - 2, y, xyLeft, y);
        char buf[8]; snprintf(buf, sizeof(buf), "%d", (int)val);
        int w = u8g2.getStrWidth(buf);
        int textX = xyLeft - 4 - w;
        if (textX < 0) textX = 0; 
        u8g2.setCursor(textX, y + 2);
        u8g2.print(buf);
      }
    }

    int lastX = -1, lastY = -1;
    for (int i = 0; i < HISTORY_LEN; i++) {
      uint8_t idx = (head + i) % HISTORY_LEN;
      float tD = ((float)distRing[idx] - dispMin) / (dispMax - dispMin);
      if (tD < 0) tD = 0; if (tD > 1) tD = 1;
      int x = xyLeft + (int)(tD * (xyRight - xyLeft));

      float currentSpeed = isRawMode ? speedRawRing[idx] : speedFiltRing[idx];
      float tS = (currentSpeed - actualSpeedMin) / (actualSpeedMax - actualSpeedMin);
      if (tS < 0) tS = 0; if (tS > 1) tS = 1;
      int y = xyBottom - (int)(tS * (xyBottom - xyTop));

      if (lastX != -1) {
        u8g2.drawLine(lastX, lastY, x, y);
      } else {
        u8g2.drawPixel(x, y);
      }
      lastX = x;
      lastY = y;
    }
    u8g2.sendBuffer();
    return;
  }

  // 都怪你都怪你都怪你

  bool isPureV = (currentMode == 3 || currentMode == 4);
  bool hasDoubleLines = (currentMode == 0 || currentMode == 1);
  
  int plotRightBound = hasDoubleLines ? 104 : 127; 
  int plotWidth = plotRightBound - PLOT_LEFT;

  if (currentMode == 0 || currentMode == 1 || currentMode == 2) {
    drawScale(dispMin, dispMax, yHigh, yLow, true);
  } else if (isPureV) {
    drawScale(actualSpeedMin, actualSpeedMax, yHigh, yLow, true);
  }
  
  if (hasDoubleLines) {
    drawScale(actualSpeedMin, actualSpeedMax, yHigh, yLow, false);
  }

  for (int i = 0; i < HISTORY_LEN; i++) {
    uint8_t idx = (head + i) % HISTORY_LEN;
    int x = PLOT_LEFT + (int)((float)i / HISTORY_LEN * plotWidth);
    
    float tD = ((float)distRing[idx] - dispMin) / (dispMax - dispMin);
    if (tD < 0) tD = 0; if (tD > 1) tD = 1;
    int yD = (uint8_t)(yLow - tD * (yLow - yHigh));

    float currentSpeed = isRawMode ? speedRawRing[idx] : speedFiltRing[idx];
    float tS = (currentSpeed - actualSpeedMin) / (actualSpeedMax - actualSpeedMin);
    if (tS < 0) tS = 0; if (tS > 1) tS = 1;
    int yS = (uint8_t)(yLow - tS * (yLow - yHigh));

    if (i < HISTORY_LEN - 1) {
      uint8_t nextIdx = (head + i + 1) % HISTORY_LEN;
      int nextX = PLOT_LEFT + (int)((float)(i + 1) / HISTORY_LEN * plotWidth);
      
      if (currentMode == 0 || currentMode == 1 || currentMode == 2) {
        float tD2 = ((float)distRing[nextIdx] - dispMin) / (dispMax - dispMin);
        if (tD2 < 0) tD2 = 0; if (tD2 > 1) tD2 = 1;
        int nyD = (uint8_t)(yLow - tD2 * (yLow - yHigh));
        u8g2.drawLine(x, yD, nextX, nyD);
      }

      if (currentMode == 0 || currentMode == 1 || isPureV) {
        float nextSpeed = isRawMode ? speedRawRing[nextIdx] : speedFiltRing[nextIdx];
        float tS2 = (nextSpeed - actualSpeedMin) / (actualSpeedMax - actualSpeedMin);
        if (tS2 < 0) tS2 = 0; if (tS2 > 1) tS2 = 1;
        int nyS = (uint8_t)(yLow - tS2 * (yLow - yHigh));
        u8g2.drawLine(x, yS, nextX, nyS);
      }
    }
  }

  u8g2.sendBuffer();
}






//芜湖~
/*
嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷嗷
      
      */