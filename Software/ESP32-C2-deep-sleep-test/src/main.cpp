#include <Arduino.h>
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_sntp.h"
#include "driver/rtc_io.h"
#include "time.h"

constexpr int SLEEP_TIME = 5;
RTC_DATA_ATTR uint16_t bootCount = 0;
RTC_DATA_ATTR bool rtc_initialized = false;

void setArbitraryTime() {
  struct tm tm;
  tm.tm_year = 2024 - 1900; // Year since 1900
  tm.tm_mon = 0;            // January
  tm.tm_mday = 1;           // 1st
  tm.tm_hour = 12;
  tm.tm_min = 0;
  tm.tm_sec = 0;
  time_t t = mktime(&tm);
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, nullptr);
}

void printCurrentTime() {
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  char buf[64];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  Serial.printf("Current RTC time: %s\n", buf);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Enter Setup");

  bootCount++;
  Serial.printf("Boot count: %d\n", bootCount);

  if (!rtc_initialized) {
    setArbitraryTime();
    rtc_initialized = true;
    Serial.println("RTC initialized to arbitrary time.");
  } else {
    Serial.println("RTC already initialized.");
  }

  printCurrentTime();

  // esp_sleep_enable_timer_wakeup(SLEEP_TIME * 1000000);
  esp_sleep_enable_ext1_wakeup(GPIO_SEL_0, ESP_EXT1_WAKEUP_ANY_HIGH);
  Serial.flush();
  esp_deep_sleep_start();
}

void loop() {
  // Not used
}
