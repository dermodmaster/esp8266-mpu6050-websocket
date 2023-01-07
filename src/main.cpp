#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESPAsyncWiFiManager.h>          // https://github.com/tzapu/WiFiManager WiFi Configuration Magic

DNSServer dns;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Adafruit_MPU6050 mpu; // Set up the MPU6050 sensor using the default I2C address (0x68)


// Websocket event handler - called on every websocket event
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
  
    Serial.println("Websocket client connection received");
     
  } else if(type == WS_EVT_DISCONNECT){
 
    Serial.println("Client disconnected");
  
  }
}

// function to light the onboard led up. LED can blink if you offer a times parameter bigger than 1.
void lightUpLED(long length, long times = 1){
  for (long i; i<times; i++){
      digitalWrite(LED_BUILTIN, LOW); // Einschalten
      delay(length);
      digitalWrite(LED_BUILTIN, HIGH); // Ausschalten
  }
}

/*
  Tries to connect to the MPU sensor. 
  Because of local I2C problems, we try all I2C adresses between 0x68 - 0x75 until the sensor is found.
  Connection state will be visualised by the onboard LED.
*/
void connectMPU(int addr){
  lightUpLED(200, 5); // begin mpu light sequence
  while (!mpu.begin(addr)) {
    if(addr > 0x75){
      // limit reached
      lightUpLED(1000);
      addr = 0x68;
    }
    else
    {
      addr++;
      lightUpLED(100);
    }
    Serial.println(String("MPU6050 not found at ")+String(addr));
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  lightUpLED(50, 5); // mpu successful light sequence
  
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT); // LED als Output definieren
  digitalWrite(LED_BUILTIN, HIGH); // Ausschalten

  // MPU6050 Sensor initialize!
  connectMPU(0x68);
  
  //first parameter is name of access point, second is the password
  AsyncWiFiManager wifiManager(&server,&dns);

  wifiManager.autoConnect("ESP-Gyrosocket");

  // Websocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  AsyncElegantOTA.begin(&server);
  server.begin();

}

void loop() {

  // Get data from the MPU6050 sensor
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Send the data to all connected WebSocket clients
  String data = String(millis())+","+String(a.acceleration.x) + "," + String(a.acceleration.y) + "," + String(a.acceleration.z) + "," + String(g.gyro.x) + "," + String(g.gyro.y) + "," + String(g.gyro.z) + "," + String(temp.temperature);
  ws.textAll(data);
}

