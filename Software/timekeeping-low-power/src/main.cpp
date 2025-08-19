#include <Arduino.h>
// #include <driver/rtc_io.h>
// #include <esp_sleep.h>
// #include <esp_system.h>
// #include <esp_sntp.h>
#include <soc/rtc.h>
#include "esp_sleep.h"
#include <esp_wifi.h>
#include "esp_private/esp_clk.h"
#include <FastLED.h>
#include <NTPClient.h>
#include <time.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>


// Contains WiFi SSID and Password
#include "secrets.h" 

// Hardware Definitions
#define LED_TYPE WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS 60
#define LED_DATA_PIN 10
#define LED_EN_PIN 2
#define SDA_PIN 18
#define SCL_PIN 19
#define IMU_INT1_PIN 3
#define IMU_INT2_PIN 9

// IMU Definitions
#define LIS2DW12_I2C_ADDRESS 0x19
#define LIS2DW12_REG_CTRL1 0x20
#define LIS2DW12_REG_CTRL2 0x21
#define LIS2DW12_REG_CTRL4_INT1_PAD_CTRL 0x23
#define LIS2DW12_REG_CTRL6 0x25
#define LIS2DW12_REG_CTRL7 0x3F
#define LIS2DW12_REG_WAKE_UP_THS 0x34
#define LIS2DW12_REG_WAKE_UP_DUR 0x35

// RTC Data Attributes
RTC_DATA_ATTR bool isRtcInitialized = false;
RTC_DATA_ATTR time_t NextWakeupTime = 0;  // Stored in RTC memory
RTC_DATA_ATTR volatile int activeTasks = 0;

void taskStarted() { activeTasks++; }
void taskEnded()   { activeTasks--; }

// Server Stuff
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

/// @brief Set the RTC time using a tm structure.
/// @param timeinfo The tm structure containing the time to set.
void setRTCTime(tm *timeinfo) {
  // Set the time using settimeofday
  time_t t = mktime(timeinfo);
  struct timeval now = { .tv_sec = t, .tv_usec = 0 };
  settimeofday(&now, nullptr);
}

/// @brief Initialize the IMU for wakeup and interrupts.
/// This function sets up the LIS2DW12 IMU to wake up on motion and generate
/// interrupts. It configures the IMU to use a low power mode and sets the
/// wakeup threshold and duration. The IMU is connected via I2C.
/// The function also sets the interrupt pin to pulse mode.
void initializeIMU() {
  // Begin Wire.
  Wire.begin(SDA_PIN, SCL_PIN);

  // TODO: May wish to replace wakeup with tap interrupt. Was more responsive during testing.
  // Ctrl 1 to 0x10. 1.6Hz, Low Power.
  Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
  Wire.write(LIS2DW12_REG_CTRL1);
  Wire.write(0x10);
  Wire.endTransmission();

  // ctrl 7 to 0x20. Enable Interrupts.
  Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
  Wire.write(LIS2DW12_REG_CTRL7);
  Wire.write(0x20);
  Wire.endTransmission();  

  // Set wakeup duration to zero? (One sample above threshold)
  Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
  Wire.write(LIS2DW12_REG_WAKE_UP_DUR);
  Wire.write(0x00);
  Wire.endTransmission();

  // A high wakeup threshold so it isn't too sensitive?
  Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
  Wire.write(LIS2DW12_REG_WAKE_UP_THS);
  Wire.write(0x0F);
  Wire.endTransmission();

  // Set Interrupt1 to pulse wakeup signal.
  Wire.beginTransmission(LIS2DW12_I2C_ADDRESS);
  Wire.write(LIS2DW12_REG_CTRL4_INT1_PAD_CTRL);
  Wire.write(0x20);
  Wire.endTransmission();

  // Configure GPIO for wakeup.
  esp_deep_sleep_enable_gpio_wakeup((1ULL << IMU_INT1_PIN), ESP_GPIO_WAKEUP_GPIO_HIGH);
}

/// @brief Schedule a weekly wakeup using the RTC timer.
/// This function sets the next wakeup time to one week from now.
void scheduleWeeklyWakeup() {
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);

  // Set next wakeup to one week from now.
  if (NextWakeupTime == 0 || now >= NextWakeupTime) {
    NextWakeupTime = now + 7 * 24 * 60 * 60;  // one week in seconds
  }

  // Calculate how many seconds until the next wakeup
  time_t seconds_until_wakeup = NextWakeupTime - now;

  // Convert to microseconds for esp_sleep_enable_timer_wakeup
  uint64_t us_until_wakeup = (uint64_t)seconds_until_wakeup * 1000000ULL;

  // Safety: if negative or zero, just force a short delay
  if (us_until_wakeup <= 0) {
      us_until_wakeup = 10 * 1000000ULL; // 10 seconds
  }

  esp_sleep_enable_timer_wakeup(us_until_wakeup);
}

/// @brief Task to show the current time on the LED strip.
/// This task runs for 5 seconds, displaying the current time on the LED strip.
/// It uses the FastLED library to control the LED strip and displays the hour, minute,
/// and second by lighting up specific LEDs in blue, green, and red respectively.
/// @param pvParameters  Parameters for the task (not used).
void ShowTimeTask(void *pvParameters) {
  taskStarted();

  setenv("TZ", "EST5EDT", 1);
  tzset();

  // Instantiate LED Strip.
  static CRGB leds[NUM_LEDS];
  FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(5);
  
  // Turn on the LED Strip. Active Low on a PMOSFET.
  // This is to save power when the LED Strip is not in use.
  pinMode(LED_EN_PIN, OUTPUT);
  digitalWrite(LED_EN_PIN, 0);

  // Clear the LED Strip.
  FastLED.clear(true);

  unsigned long startTime = millis();

  while (startTime + 5000 > millis()) { // Run for 5 seconds
    // Base the time on epoch. This allows DST To work automatically.
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    // Set the LED color based on the current time
    uint8_t hour = timeinfo.tm_hour;
    uint8_t minute = timeinfo.tm_min;
    uint8_t second = timeinfo.tm_sec;

    // Calculate the indices for hour, minute, and second
    // 24 hour format.
    uint8_t hourIndex = 59 - ((((hour * 60 + minute) / 24) + 29) % NUM_LEDS);
    uint8_t minuteIndex = 59 - ((minute + 29) % NUM_LEDS);
    uint8_t secondIndex = 59 - ((second + 29) % NUM_LEDS);
    
    // Set all LEDs to black
    FastLED.clear();
    leds[secondIndex] += CRGB::Red;
    leds[minuteIndex] += CRGB::Green;
    leds[hourIndex] += CRGB::Blue;
    FastLED.show();

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  // Turn off the LED Strip.
  FastLED.clear(true);
  pinMode(LED_EN_PIN, INPUT);

  // Delete this task.
  taskEnded();
  vTaskDelete(NULL);
}

/// @brief Set the time using NTP and WiFi.
/// This task connects to WiFi, retrieves the current time from an NTP server,
/// and sets the RTC time. It runs for a maximum of 2 minutes to connect to WiFi.
/// If successful, it sets the RTC time and disconnects WiFi to save power.
/// If it fails to connect, it will not set the time.
/// @param pvParameters Parameters for the task (not used).
void SetTimeTask(void *pvParameters) {
  taskStarted();
  // Connect to WiFi within 2 minutes.
  WiFi.begin(ssid, password);
  unsigned long timeout = millis() + 120000; // 2 minute timeout
  while (!WiFi.isConnected() && millis() < timeout) {
    Serial.println("Waiting for WiFi connection...");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  // If we've connected, grab NTP time and set the RTC.
  if (WiFi.isConnected()) {
    Serial.println("WiFi connected. Setting time...");
    // Set timezone to EST/EDT
    setenv("TZ", "EST5EDT", 1);
    tzset();
    // Initialize NTP Client
    timeClient.begin();
    for (int i = 0; i < 5 && !timeClient.update(); i++) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    // Get epoch time and set RTC
    time_t now = timeClient.getEpochTime();
    struct tm *timeinfo = localtime(&now);
    setRTCTime(timeinfo);
    isRtcInitialized = true;
    Serial.println("Time set successfully.");
  } else {
    Serial.println("Failed to connect to WiFi. Time not set.");
  }

  // Disconnect WiFi to save power.
  esp_wifi_stop();

  Serial.println("WiFi disconnected.");  

  // Delete this task.
  taskEnded();
  vTaskDelete(NULL);
}


#define CALIBRATE_ONE(cali_clk) calibrate_one(cali_clk, #cali_clk)

static uint32_t calibrate_one(rtc_cal_sel_t cal_clk, const char *name)
{

    const uint32_t cal_count = 1000;
    const float factor = (1 << 19) * 1000.0f;
    uint32_t cali_val;
    printf("%s:\n", name);
    for (int i = 0; i < 5; ++i)
    {
        printf("calibrate (%d): ", i);
        cali_val = rtc_clk_cal(cal_clk, cal_count);
        printf("%.3f kHz\n", factor / (float)cali_val);
    }
    return cali_val;
}

/// @brief Setup function for the ESP32-C3 wristwatch.
/// This function initializes the LED strip, IMU, and schedules the next wakeup.
/// It also creates tasks for setting the time and showing the current time.
/// It checks if the RTC has been initialized and sets the time if it hasn't.
/// If the device is woken up by the IMU, it shows the current time.
/// Finally, it goes to deep sleep until the next scheduled wakeup.
void setup() { 
  rtc_clk_32k_bootstrap(512);
  rtc_clk_32k_bootstrap(512);
  rtc_clk_32k_enable(true);

  uint32_t cal_32k = CALIBRATE_ONE(RTC_CAL_32K_XTAL);
  rtc_clk_slow_freq_set(RTC_SLOW_FREQ_32K_XTAL);

  Serial.begin(115200);

  if (cal_32k == 0) {
      printf("32K XTAL OSC has not started up");
  } else {
      printf("done\n");
  }

  if (rtc_clk_32k_enabled()) {
      Serial.println("OSC Enabled");
  }
  
  // If firstboot:
  if (!isRtcInitialized || esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    xTaskCreate(SetTimeTask, "Set Time Task", 4096, NULL, 1, NULL);
  }

  // If woken up by the IMU, show the time.
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO) {
    Serial.println("Woke up from GPIO interrupt. Showing time...");
    xTaskCreate(ShowTimeTask, "Show Time Task", 2048, NULL, 1, NULL);
  }
  
  // Initialize the IMU for wakeup.
  initializeIMU();

  // Schedule the next weekly wakeup.
  scheduleWeeklyWakeup();

  while (activeTasks > 0) {
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  
  // Go to sleep now.waits for it with a timeout
  Serial.flush();
  esp_deep_sleep_start();
}

/// @brief Main loop function.
/// This function is empty as all tasks are handled in the setup and task functions.
/// It is included to satisfy the Arduino framework requirements.
void loop() {
  // Nothing to do here. Everything is RTC, ULP, and Task based.
}