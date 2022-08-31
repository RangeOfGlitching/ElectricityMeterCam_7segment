/**
 * The MIT License (MIT)
 * 
 * Copyright (c) 2021 Marc Roßbach
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#define DEBUG
// #define mqtt
#define LoRa
// #define FlashLight
#define softap

#include <Arduino.h>
#include "soc/soc.h"          //disable brownout problems
#include "soc/rtc_cntl_reg.h" //disable brownout problems
#include "CameraServer.h"
#include "OCR.h"
#include "Settings.h"
#include "WebServer.h"

#define LED_PIN 4
#define MIN_CONFIDENCE 0.4f

#ifdef mqtt
#include "NTPClient.h"
#include "WifiHelper.h"
#include "PubSubClient.h"
#include "wifi_config.h"
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
#endif

#ifdef softap
#include "softap_helper.h"
// #include "softap_config.h"
// Set web server port number to 80
// WiFiServer server(80);
#endif

Settings settings;
OCR *ocr;
CameraServer camServer(settings);

int DetectDigit(dl_matrix3du_t* frame, const int x, const int y, const int width, const int height, float* confidence);
KwhInfo AnalyzeFrame(dl_matrix3du_t* frame);
void taskDelay(unsigned long milisec);
#ifdef mqtt
void mqttUpdate();
void updateConnections();
#endif

#ifdef LoRa
#include "RYLR896.h"
RYLR896* lora;
HardwareSerial LoraSerial(1);
#endif

void setup()
{
    //disable brownout detector
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
    Serial.begin(115200);



    #ifdef DEBUG
    Serial.println("starting ...");
    #endif

    settings.Load();


#ifdef mqtt
    WifiHelper::Connect();
#endif

#ifdef softap
    softapHelper::Connect();
#endif
    if (camServer.InitCamera(false))
    {
        camServer.StartServer();

#ifdef FlashLight
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, LOW);
#endif

#ifdef DEBUG
        Serial.println("started");
#endif   
    }
#ifdef LoRa
    hardSerialConfig_t config = {
    .baudRate = 115200,
    .rxPin = 14,
    .txPin = 2,
    .serialMode = SERIAL_8N1};
    lora = new RYLR896(&LoraSerial, config);
    taskDelay(100);
    lora->SetRFParamsLessThan3KM();
    taskDelay(100);
    lora->SetAddress(115);
    taskDelay(100);
    lora->SetNetworkID(6);
    taskDelay(100);
    lora->SetBand(915000000);
    taskDelay(100);
#endif

    # ifdef mqtt
    mqttClient.setServer(CLOUD, 1883);
    # endif
    ocr = new OCR(ocr_model_28x28_c11_tflite, 28, 28, 11);
}

void loop()
{
    #ifdef mqtt
    updateConnections();
    #endif
    auto* frame = camServer.CaptureFrame(); 
    
    if (frame != nullptr)
    {
        #ifdef DEBUG
        Serial.println("Auswertung");
        #endif

        KwhInfo info = AnalyzeFrame(frame);

        camServer.SetLatestKwh(info);

        // send frame to http://esp32cam/ endpoint
        camServer.SwapBuffers();
        
        #ifdef mqtt
        // send tp MQTT server
        mqttClient.publish("metercam/confidence", String(info.confidence * 100, 0).c_str());
        mqttClient.publish("metercam/metervalue", info.result.c_str());
        // if (info.confidence > 0.6)
        #endif

        #ifdef LoRa
        lora->Send(info.result + String(info.confidence * 100, 0), 116);
        #endif
    }

    if (millis() < 300000) // more frequent updates in first 5 minutes
    {
        taskDelay(500);
    }
    else
    {
        for (int i = 0; i < 120 && !camServer.UserConnected(); i++)
        {
            taskDelay(500);
        }
    }    

    if (millis() > 24 * 60 * 60 * 1000) // restart esp after 24 hours
    {
        Serial.println("Restart");
        Serial.flush();
        ESP.restart();
    }
}

int DetectDigit(dl_matrix3du_t* frame, const int x, const int y, const int width, const int height, float* confidence)
{
    int digit = ocr->PredictDigit(frame, x, y, width, height, confidence);
    uint32_t color = ImageUtils::GetColorFromConfidence(*confidence, MIN_CONFIDENCE, 1.0f);
    ImageUtils::DrawRect(x, y, width, height, color, frame);
    ImageUtils::DrawFillRect(x, y - 4, width * (*confidence), 4, color, frame);
    ImageUtils::DrawText(x + width / 5, y + height, color, String(digit), frame);
    return digit;
}


KwhInfo AnalyzeFrame(dl_matrix3du_t* frame)
{
    KwhInfo info = {};
    info.kwh = 0;
    info.confidence = 1.0;

    for (int i = 0; i < NUM_DIGITS; i++)
    {
        const DigitBBox bbox = settings.GetDigitBBox(i);
        if (bbox.w > 0 && bbox.h > 0)
        {
            float conf = 0;
            const int digit = DetectDigit(frame, bbox.x, bbox.y, bbox.w, bbox.h, &conf);
            info.confidence = std::min(conf, info.confidence);
            info.kwh += pow(10, 5 - i) * digit;
            if (i==3){
                info.result += " ";
            }
            info.result += String(digit);
            Serial.println("Kwh " + String(info.result));
        }
    }

    uint32_t color = ImageUtils::GetColorFromConfidence(info.confidence, MIN_CONFIDENCE, 1.0f);
    ImageUtils::DrawText(120, 5, color, String("") +  (int)(info.confidence * 100) + "%", frame);

    Serial.println(String(" VALUE: ") + info.kwh + " kWh (" + (info.confidence * 100) + "%)");

    return info;
}

void taskDelay(unsigned long milisec)
{
    vTaskDelay(milisec);
}

#ifdef mqtt
void mqttUpdate()
{
    if (strlen(CLOUD) > 0)
    {
        for (int i = 0; i < 5 && !mqttClient.connected(); ++i)
        {
            Serial.print("Attempting MQTT connection...");

            if (mqttClient.connect("metercam"))
            {
                Serial.println("connected");
            }
            else
            {
                Serial.print("failed, rc=");
                Serial.print(mqttClient.state());
                Serial.println(" try again in 1 seconds");
                taskDelay(1000);
            }
        }

        mqttClient.loop();
    }
}
#endif
void updateConnections()
{
    #ifdef mqtt
    if (!WiFi.isConnected())
    {
        WifiHelper::Connect();
    }
    
    mqttUpdate();
    #endif
}