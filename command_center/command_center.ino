///////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) Attunix, 2016. All rights reserved.
//
// This example code shows how to use an Adafruit Feather M0 module and Microsoft Azure for IoT
// applications.

#include "iot_configs.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <SPI.h>
#include <WiFi101.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "rem_ctrl.h"
#include "NTPClient.h"
#include <AzureIoTHub.h>
#include <AzureIoTProtocol_MQTT.h>

#ifdef ARDUINO_SAMD_FEATHER_M0
#define WINC_CS   8
#define WINC_IRQ  7
#define WINC_RST  4
#define WINC_EN   2

#endif


static const char ssid[] = IOT_CONFIG_WIFI_SSID;
static const char pass[] = IOT_CONFIG_WIFI_PASSWORD;
static const char* connectionString = IOT_CONFIG_CONNECTION_STRING;

// The connection string is the one which begins with HostName=... and contains the DeviceId
// and SharedAccessKey for this particular Thing on the Internet.

int status = WL_IDLE_STATUS;


#define SEALEVELPRESSURE_HPA (1013.25)
const int Bme280_cs_pin__i = 5;
bool Bme_init_result = false;
Adafruit_BME280 bme(Bme280_cs_pin__i);
void initTime();


void setup() {
  // The Feather M0 loses it's COMn connection with every reset.
  // This 10 s delay allows you to reselect the COM port and open the serial monitor window.
  delay(10000);
  
  Serial.begin(9600);

#ifdef ARDUINO_SAMD_FEATHER_M0
    //Configure pins for Adafruit ATWINC1500 Feather
    Serial.println(F("WINC1500 on FeatherM0 detected."));
    Serial.println(F("Setting pins for WiFi101 library (WINC1500 on FeatherM0)"));
    WiFi.setPins(WINC_CS, WINC_IRQ, WINC_RST, WINC_EN);
    // for the Adafruit WINC1500 we need to enable the chip
    pinMode(WINC_EN, OUTPUT);
    digitalWrite(WINC_EN, HIGH);
    Serial.println(F("Enabled WINC1500 interface for FeatherM0"));
#endif

  Serial.println("Checking for the presence of the BME280 temp/humid/press module.");
  Bme_init_result = bme.begin();
  if (Bme_init_result)
  {
    Serial.println("Found and initialized BME280 module.");
  }
  else
  {
    Serial.println("Warning! BME280 module not found.");
  }
  
  Serial.println("Checking for the presence of the WiFi module.");
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("Warning! WiFi shield not found.");
  }
  else
  {
    Serial.println("WiFi module found.");
  }

  Serial.println("Attempting to connect to Wifi network.");
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    if (status != WL_CONNECTED) {
      // wait 10 seconds for connection:
      delay(10000);
    }
  }
  Serial.println("Connected to wifi");

  if (rem_ctrl_set_connection_string(connectionString)) {
    Serial.print("Connection string successfully set to \"");
    Serial.print(connectionString);
    Serial.println("\"");
  }
  else {
    Serial.println("Unable to set connection string. (too long?)");
  }

  int Init_result__i = rem_ctrl_init();
  if (Init_result__i < 4)
  {
    Serial.println("Unable to initialize the Azure connection. Halting.");
  }

  initTime();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// The Arduino millisecond clock is used here to get a simplistic multitasking effect.
// This macro handles wrap-around correctly.
#define IS_TASK_TIME(curr_time_ms, next_task_time_ms) \
  ((curr_time_ms >= next_task_time_ms) && ( ! ((curr_time_ms ^ next_task_time_ms) & 0x80000000L)))
uint32_t const Sensor_read_period_ms__u32 = 15000;
uint32_t Sensor_read_next_ms__u32 = 0;
uint32_t const Azure_io_update_period_ms__u32 = 100;
uint32_t Azure_io_update_next_ms__u32 = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  uint32_t Curr_time_ms__u32 = millis();

  if (IS_TASK_TIME(Curr_time_ms__u32, Sensor_read_next_ms__u32)) {
    float Temp_c__f = bme.readTemperature();
    float Pres_hPa__f = bme.readPressure() / 100;
    float Humi_pct__f = bme.readHumidity();
    printf("Temp=%.2f, Pres=%.2f, Humi=%.2f\n", Temp_c__f, Pres_hPa__f, Humi_pct__f);
    rem_ctrl_send_data(Temp_c__f, Pres_hPa__f, Humi_pct__f);

    Sensor_read_next_ms__u32 = Curr_time_ms__u32 + Sensor_read_period_ms__u32;
  }
  
  if (IS_TASK_TIME(Curr_time_ms__u32, Azure_io_update_next_ms__u32)) {
    rem_ctrl_run();

    Azure_io_update_next_ms__u32 = Curr_time_ms__u32 + Azure_io_update_period_ms__u32;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void initTime() {
    WiFiUDP     _udp;

    time_t epochTime = (time_t)-1;

    NTPClient ntpClient;
    ntpClient.begin();

    while (true) {
        epochTime = ntpClient.getEpochTime("0.pool.ntp.org");

        if (epochTime == (time_t)-1) {
            Serial.println("Fetching NTP epoch time failed! Waiting 5 seconds to retry.");
            delay(5000);
        } else {
            Serial.print("Fetched NTP epoch time is: ");
            Serial.println(epochTime);
            break;
        }
    }
    
    ntpClient.end();

    struct timeval tv;
    tv.tv_sec = epochTime;
    tv.tv_usec = 0;

    settimeofday(&tv, NULL);
}


