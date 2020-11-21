#include "CameraServer.h"
#include <Arduino.h>
#include "esp_http_server.h"
#include "camera_pins.h"

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
httpd_handle_t indexHtmlHandler = nullptr;
SemaphoreHandle_t bufferSemaphore = xSemaphoreCreateMutex();
dl_matrix3du_t* httpFrontBuffer = nullptr;

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    while (true)
    {
        if (httpFrontBuffer == nullptr)
        {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        }
        else
        {
            xSemaphoreTakeRecursive(bufferSemaphore, portMAX_DELAY);
            {
                Serial.println("Compressing RGB to JPEG");
                if (!fmt2jpg(
                        httpFrontBuffer->item, 
                        httpFrontBuffer->stride * httpFrontBuffer->h,
                        httpFrontBuffer->w,
                        httpFrontBuffer->h,
                        PIXFORMAT_RGB888,
                        80, &_jpg_buf, &_jpg_buf_len))
                {
                    Serial.println("JPEG compression failed");
                    res = ESP_FAIL;
                }
            }
            xSemaphoreGiveRecursive(bufferSemaphore);
        }

        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }

        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }

        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }

        if (_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }

        if (res != ESP_OK)
        {
            break;
        }

        //Serial.printf("MJPG: %uB\n", (uint32_t)(_jpg_buf_len));
    }

    return res;
}

CameraServer::CameraServer() :
    _frontRgbBuffer(nullptr),
    _backRgbBuffer(nullptr)
{
}

CameraServer::~CameraServer()
{
    if (_frontRgbBuffer != nullptr)
        dl_matrix3du_free(_frontRgbBuffer);

    if (_backRgbBuffer != nullptr)
        dl_matrix3du_free(_backRgbBuffer);
}

bool CameraServer::StartServer()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.task_priority = 1;
    config.server_port = 80;

    httpd_uri_t indexUri = {.uri = "/",
                             .method = HTTP_GET,
                             .handler = stream_handler,
                             .user_ctx = NULL};

    //Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&indexHtmlHandler, &config) == ESP_OK)
    {
        httpd_register_uri_handler(indexHtmlHandler, &indexUri);
        return true;
    }

    return false;
}

bool CameraServer::InitCamera(const bool flipImage)
{
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
        Serial.println();
        return false;
    }

    if (flipImage)
    {
        sensor_t *s = esp_camera_sensor_get();
        s->set_vflip(s, 1);   //flip vertically
        s->set_hmirror(s, 1); //flip horizontally
    }

    return true;
}

dl_matrix3du_t* CameraServer::CaptureFrame()
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == nullptr)
    {
        Serial.println("Camera capture failed");
        return nullptr;
    }

    if (_backRgbBuffer == nullptr)
    {
        _frontRgbBuffer = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
        memset(_frontRgbBuffer->item, 255, _frontRgbBuffer->stride * _frontRgbBuffer->h);

        _backRgbBuffer = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
        memset(_backRgbBuffer->item, 255, _backRgbBuffer->stride * _backRgbBuffer->h);
    }

    xSemaphoreTakeRecursive(bufferSemaphore, portMAX_DELAY);
    {
        std::swap(_frontRgbBuffer, _backRgbBuffer);
        httpFrontBuffer = _frontRgbBuffer;
    }
    xSemaphoreGiveRecursive(bufferSemaphore);

    if (!fmt2rgb888(fb->buf, fb->len, fb->format, _backRgbBuffer->item))
    {
        Serial.println("fmt2rgb888 failed");
    }

    if (fb != nullptr)
    {
        esp_camera_fb_return(fb);
    }

    return _backRgbBuffer;
}