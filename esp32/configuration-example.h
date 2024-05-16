#pragma once

#define WIFI_SSID               "WIFI_SSID"
#define WIFI_PASSWORD           "WIFI_PASSWORD"
#define WIFI_RETRY_INTERVAL     1000                        // milliseconds
#define MQTT_SERVER             "MQTT_SERVER_IP_ADDRESS"
#define MQTT_SERVER_PORT        1883
#define MQTT_CLIENT_ID          "MQTT_CLIENT_ID"            // example: ESP32-room1
#define MQTT_MOTION_TOPIC       "MQTT_MOTION_TOPIC"         // example: room1/motions
#define MQTT_PING_TOPIC         "MQTT_PING_TOPIC"           // example: pings
#define MQTT_RETRY_INTERVAL     5000                        // milliseconds
#define MQTT_PING_INTERVAL      50000                       // milliseconds
#define MQTT_PUBLISH_MAX_TRIES  12
#define MOTION_SENSOR_PIN       13
#define MOTION_SENSOR_WARM_UP   20000                       // milliseconds