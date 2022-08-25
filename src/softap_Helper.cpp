#include <Arduino.h>
#include "softap_config.h"
#include "softap_helper.h"
#define DEBUG
void softapHelper::Connect(){
    WiFi.softAP(softap_ssid, softap_password);
    IPAddress IP = WiFi.softAPIP();
    #ifdef DEBUG
    Serial.print("AP IP address: http://");
    Serial.println(IP);
    #endif
}