#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

#include <FastLED.h>
#include <NTPClient.h>

#include "secrets.h" // Contains WiFi SSID and Password

#define NUM_LEDS 60
#define LED_DATA_PIN 10
#define LED_EN_PIN 2
#define SDA_PIN 18
#define SCL_PIN 19

// Server Stuff
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void LEDTask(void *pvParameters) {
  // Instantiate LED Strip.
  CRGB leds[NUM_LEDS];
  FastLED.addLeds<WS2812, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(5);
  
  // Turn on the LED Strip.
  pinMode(LED_EN_PIN, OUTPUT);
  digitalWrite(LED_EN_PIN, 0);

  // Clear the LED Strip.
  FastLED.clear(true);

  while (1) {
    // Base the time on epoch. This allows DST To work automatically.
    time_t now = timeClient.getEpochTime();
    struct tm *timeinfo = localtime(&now);

    // Set the LED color based on the current time
    uint8_t hour = timeinfo->tm_hour;
    uint8_t minute = timeinfo->tm_min;
    uint8_t second = timeinfo->tm_sec;

    // Calculate the indices for hour, minute, and second
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
}



void setup() {  
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed!");
  }

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  setenv("TZ", "EST5EDT", 1);
  tzset();
  
  // Begin Time Keeping
  timeClient.begin();
  timeClient.update();

  xTaskCreate(LEDTask, "LED Task", 1024, NULL, 1, NULL);

}

void loop() {
  ArduinoOTA.handle();
}