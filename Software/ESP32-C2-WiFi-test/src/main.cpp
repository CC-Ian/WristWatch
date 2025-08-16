#include <Arduino.h>
#include <WiFi.h>

void setup() {  
  Serial.begin(115200);
  while (! Serial.available()) {
    delay(10);
  }
  Serial.println("Watch Is on!");

  WiFi.begin("SSID", "PSK");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP()); 
}

void loop() {
  delay(100);
  Serial.println("WiFi RSSI: " + String(WiFi.RSSI()));
}