//Filename: app_camera/include/app_camera.hpp
#pragma once

#include <list>

#include "esp_camera.h"

#include "__base__.hpp"

#define CONFIG_CAMERA_MODULE_ESP_S3_EYE 1

#if CONFIG_CAMERA_MODULE_WROVER_KIT
#define CAMERA_MODULE_NAME "Wrover Kit"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_XCLK 21
#define CAMERA_PIN_SIOD 26
#define CAMERA_PIN_SIOC 27

#define CAMERA_PIN_D7 35
#define CAMERA_PIN_D6 34
#define CAMERA_PIN_D5 39
#define CAMERA_PIN_D4 36
#define CAMERA_PIN_D3 19
#define CAMERA_PIN_D2 18
#define CAMERA_PIN_D1 5
#define CAMERA_PIN_D0 4
#define CAMERA_PIN_VSYNC 25
#define CAMERA_PIN_HREF 23
#define CAMERA_PIN_PCLK 22

#elif CONFIG_CAMERA_MODULE_ESP_EYE
#define CAMERA_MODULE_NAME "ESP-EYE"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_XCLK 4
#define CAMERA_PIN_SIOD 18
#define CAMERA_PIN_SIOC 23

#define CAMERA_PIN_D7 36
#define CAMERA_PIN_D6 37
#define CAMERA_PIN_D5 38
#define CAMERA_PIN_D4 39
#define CAMERA_PIN_D3 35
#define CAMERA_PIN_D2 14
#define CAMERA_PIN_D1 13
#define CAMERA_PIN_D0 34
#define CAMERA_PIN_VSYNC 5
#define CAMERA_PIN_HREF 27
#define CAMERA_PIN_PCLK 25

#elif CONFIG_CAMERA_MODULE_ESP_S2_KALUGA
#define CAMERA_MODULE_NAME "ESP-S2-KALUGA"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_XCLK 1
#define CAMERA_PIN_SIOD 8
#define CAMERA_PIN_SIOC 7

#define CAMERA_PIN_D7 38
#define CAMERA_PIN_D6 21
#define CAMERA_PIN_D5 40
#define CAMERA_PIN_D4 39
#define CAMERA_PIN_D3 42
#define CAMERA_PIN_D2 41
#define CAMERA_PIN_D1 37
#define CAMERA_PIN_D0 36
#define CAMERA_PIN_VSYNC 2
#define CAMERA_PIN_HREF 3
#define CAMERA_PIN_PCLK 33

#elif CONFIG_CAMERA_MODULE_ESP_S3_EYE
#define CAMERA_MODULE_NAME "ESP-S3-EYE"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET -1

#define CAMERA_PIN_VSYNC 6
#define CAMERA_PIN_HREF 7
#define CAMERA_PIN_PCLK 13
#define CAMERA_PIN_XCLK 15

#define CAMERA_PIN_SIOD 4
#define CAMERA_PIN_SIOC 5

#define CAMERA_PIN_D0 11
#define CAMERA_PIN_D1 9
#define CAMERA_PIN_D2 8
#define CAMERA_PIN_D3 10
#define CAMERA_PIN_D4 12
#define CAMERA_PIN_D5 18
#define CAMERA_PIN_D6 17
#define CAMERA_PIN_D7 16

#elif CONFIG_CAMERA_MODEL_ESP32_CAM_BOARD
#define CAMERA_MODULE_NAME "ESP-DEVCAM"
#define CAMERA_PIN_PWDN 32
#define CAMERA_PIN_RESET 33

#define CAMERA_PIN_XCLK 4
#define CAMERA_PIN_SIOD 18
#define CAMERA_PIN_SIOC 23

#define CAMERA_PIN_D7 36
#define CAMERA_PIN_D6 19
#define CAMERA_PIN_D5 21
#define CAMERA_PIN_D4 39
#define CAMERA_PIN_D3 35
#define CAMERA_PIN_D2 14
#define CAMERA_PIN_D1 13
#define CAMERA_PIN_D0 34
#define CAMERA_PIN_VSYNC 5
#define CAMERA_PIN_HREF 27
#define CAMERA_PIN_PCLK 25

#elif CONFIG_CAMERA_MODULE_M5STACK_PSRAM
#define CAMERA_MODULE_NAME "M5STACK-PSRAM"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET 15

#define CAMERA_PIN_XCLK 27
#define CAMERA_PIN_SIOD 25
#define CAMERA_PIN_SIOC 23

#define CAMERA_PIN_D7 19
#define CAMERA_PIN_D6 36
#define CAMERA_PIN_D5 18
#define CAMERA_PIN_D4 39
#define CAMERA_PIN_D3 5
#define CAMERA_PIN_D2 34
#define CAMERA_PIN_D1 35
#define CAMERA_PIN_D0 32
#define CAMERA_PIN_VSYNC 22
#define CAMERA_PIN_HREF 26
#define CAMERA_PIN_PCLK 21

#elif CONFIG_CAMERA_MODULE_M5STACK_WIDE
#define CAMERA_MODULE_NAME "M5STACK-WIDE"
#define CAMERA_PIN_PWDN -1
#define CAMERA_PIN_RESET 15
#define CAMERA_PIN_XCLK 27
#define CAMERA_PIN_SIOD 22
#define CAMERA_PIN_SIOC 23

#define CAMERA_PIN_D7 19
#define CAMERA_PIN_D6 36
#define CAMERA_PIN_D5 18
#define CAMERA_PIN_D4 39
#define CAMERA_PIN_D3 5
#define CAMERA_PIN_D2 34
#define CAMERA_PIN_D1 35
#define CAMERA_PIN_D0 32
#define CAMERA_PIN_VSYNC 25
#define CAMERA_PIN_HREF 26
#define CAMERA_PIN_PCLK 21

#elif CONFIG_CAMERA_MODULE_AI_THINKER
#define CAMERA_MODULE_NAME "AI-THINKER"
#define CAMERA_PIN_PWDN 32
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_XCLK 0
#define CAMERA_PIN_SIOD 26
#define CAMERA_PIN_SIOC 27

#define CAMERA_PIN_D7 35
#define CAMERA_PIN_D6 34
#define CAMERA_PIN_D5 39
#define CAMERA_PIN_D4 36
#define CAMERA_PIN_D3 21
#define CAMERA_PIN_D2 19
#define CAMERA_PIN_D1 18
#define CAMERA_PIN_D0 5
#define CAMERA_PIN_VSYNC 25
#define CAMERA_PIN_HREF 23
#define CAMERA_PIN_PCLK 22

#elif CONFIG_CAMERA_MODULE_CUSTOM
#define CAMERA_MODULE_NAME "CUSTOM"
#define CAMERA_PIN_PWDN CONFIG_CAMERA_PIN_PWDN
#define CAMERA_PIN_RESET CONFIG_CAMERA_PIN_RESET
#define CAMERA_PIN_XCLK CONFIG_CAMERA_PIN_XCLK
#define CAMERA_PIN_SIOD CONFIG_CAMERA_PIN_SIOD
#define CAMERA_PIN_SIOC CONFIG_CAMERA_PIN_SIOC

#define CAMERA_PIN_D7 CONFIG_CAMERA_PIN_Y9
#define CAMERA_PIN_D6 CONFIG_CAMERA_PIN_Y8
#define CAMERA_PIN_D5 CONFIG_CAMERA_PIN_Y7
#define CAMERA_PIN_D4 CONFIG_CAMERA_PIN_Y6
#define CAMERA_PIN_D3 CONFIG_CAMERA_PIN_Y5
#define CAMERA_PIN_D2 CONFIG_CAMERA_PIN_Y4
#define CAMERA_PIN_D1 CONFIG_CAMERA_PIN_Y3
#define CAMERA_PIN_D0 CONFIG_CAMERA_PIN_Y2
#define CAMERA_PIN_VSYNC CONFIG_CAMERA_PIN_VSYNC
#define CAMERA_PIN_HREF CONFIG_CAMERA_PIN_HREF
#define CAMERA_PIN_PCLK CONFIG_CAMERA_PIN_PCLK
#endif

#define XCLK_FREQ_HZ 15000000

class AppCamera : public Frame
{
public:
    AppCamera(const pixformat_t pixel_fromat,
              const framesize_t frame_size,
              const uint8_t fb_count,
              QueueHandle_t queue_o = nullptr);

    void run();
};
//Filename: app_camera/include/__base__.hpp
#pragma once

#include <list>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_camera.h"

typedef enum
{
    COMMAND_TIMEOUT = -2,
    COMMAND_NOT_DETECTED = -1,

    MENU_STOP_WORKING = 0,
    MENU_DISPLAY_ONLY = 1,
    MENU_FACE_RECOGNITION = 2,
    MENU_MOTION_DETECTION = 3,

    ACTION_ENROLL = 4,
    ACTION_DELETE = 5,
    ACTION_RECOGNIZE = 6
} command_word_t;

class Observer
{
public:
    virtual void update() = 0;
};

class Subject
{
private:
    std::list<Observer *> observers;

public:
    void attach(Observer *observer)
    {
        this->observers.push_back(observer);
    }

    void detach(Observer *observer)
    {
        this->observers.remove(observer);
    }

    void detach_all()
    {
        this->observers.clear();
    }

    void notify()
    {
        for (auto observer : this->observers)
            observer->update();
    }
};

class Frame
{
public:
    QueueHandle_t queue_i;
    QueueHandle_t queue_o;
    void (*callback)(camera_fb_t *);

    Frame(QueueHandle_t queue_i = nullptr,
          QueueHandle_t queue_o = nullptr,
          void (*callback)(camera_fb_t *) = nullptr) : queue_i(queue_i),
                                                       queue_o(queue_o),
                                                       callback(callback) {}

    void set_io(QueueHandle_t queue_i, QueueHandle_t queue_o)
    {
        this->queue_i = queue_i;
        this->queue_o = queue_o;
    }
};
//Filename: app_camera/src/app_camera.cpp
#include "app_camera.hpp"

#include "esp_log.h"
#include "esp_system.h"

const static char TAG[] = "App/Camera";

AppCamera::AppCamera(const pixformat_t pixel_fromat,
                     const framesize_t frame_size,
                     const uint8_t fb_count,
                     QueueHandle_t queue_o) : Frame(nullptr, queue_o, nullptr)
{
    ESP_LOGI(TAG, "Camera module is %s", CAMERA_MODULE_NAME);

#if CONFIG_CAMERA_MODEL_ESP_EYE || CONFIG_CAMERA_MODEL_ESP32_CAM_BOARD
    /* IO13, IO14 is designed for JTAG by default,
     * to use it as generalized input,
     * firstly declair it as pullup input */
    gpio_config_t conf;
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.pin_bit_mask = 1LL << 13;
    gpio_config(&conf);
    conf.pin_bit_mask = 1LL << 14;
    gpio_config(&conf);
    ESP_LOGI(TAG, "We're in the ESP_EYE or ESP32_CAM_BOARD");
#endif

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAMERA_PIN_D0;
    config.pin_d1 = CAMERA_PIN_D1;
    config.pin_d2 = CAMERA_PIN_D2;
    config.pin_d3 = CAMERA_PIN_D3;
    config.pin_d4 = CAMERA_PIN_D4;
    config.pin_d5 = CAMERA_PIN_D5;
    config.pin_d6 = CAMERA_PIN_D6;
    config.pin_d7 = CAMERA_PIN_D7;
    config.pin_xclk = CAMERA_PIN_XCLK;
    config.pin_pclk = CAMERA_PIN_PCLK;
    config.pin_vsync = CAMERA_PIN_VSYNC;
    config.pin_href = CAMERA_PIN_HREF;
    config.pin_sscb_sda = CAMERA_PIN_SIOD;
    config.pin_sscb_scl = CAMERA_PIN_SIOC;
    config.pin_pwdn = CAMERA_PIN_PWDN;
    config.pin_reset = CAMERA_PIN_RESET;
    config.xclk_freq_hz = XCLK_FREQ_HZ;
    config.pixel_format = pixel_fromat;
    config.frame_size = frame_size;
    config.jpeg_quality = 12;
    config.fb_count = fb_count;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1); // flip it back
    
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID)
    {
        s->set_brightness(s, 1);  // up the blightness just a bit
        s->set_saturation(s, -2); // lower the saturation
    }
    s->set_sharpness(s, 2);
    s->set_awb_gain(s, 2);
}

static void task(AppCamera *self)
{
    ESP_LOGI(TAG, "Start");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (true)
    {
        if (self->queue_o == nullptr)
            break;

        camera_fb_t *frame = esp_camera_fb_get();
        
        if (frame) {
            ESP_LOGI(TAG, "Received frame from camera");
            xQueueSend(self->queue_o, &frame, portMAX_DELAY);
        }

        if(frame) {
            ESP_LOGI(TAG, "Frame received is of dimensions %dx%d", frame->width, frame->height);
        }
    }

    ESP_LOGD(TAG, "Stop");
    vTaskDelete(NULL);
}

void AppCamera::run()
{
    xTaskCreatePinnedToCore((TaskFunction_t)task, TAG, 3 * 1024, this, 5, NULL, 0);
}
//Filename: app_lcd/include/app_lcd.hpp
#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "app_camera.hpp"

#define BOARD_LCD_MOSI 47
#define BOARD_LCD_MISO -1
#define BOARD_LCD_SCK 21
#define BOARD_LCD_CS 44
#define BOARD_LCD_DC 43
#define BOARD_LCD_RST -1
#define BOARD_LCD_BL 48
#define BOARD_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define BOARD_LCD_BK_LIGHT_ON_LEVEL 0
#define BOARD_LCD_BK_LIGHT_OFF_LEVEL !BOARD_LCD_BK_LIGHT_ON_LEVEL
#define BOARD_LCD_H_RES 240
#define BOARD_LCD_V_RES 240
#define BOARD_LCD_CMD_BITS 8
#define BOARD_LCD_PARAM_BITS 8
// #define LCD_HOST SPI2_HOST

class AppLCD : public Observer, public Frame
{
private:

public:
    esp_lcd_panel_handle_t panel_handle;
    bool switch_on;
    bool paper_drawn;

    AppLCD(QueueHandle_t xQueueFrameI = nullptr,
           QueueHandle_t xQueueFrameO = nullptr,
           void (*callback)(camera_fb_t *) = esp_camera_fb_return);

    void draw_wallpaper();
    void draw_color(int color);

    void update();

    void run();
};
//Filename: app_lcd/include/image_utils.hpp
// Filename: components/app_lcd/include/image_utils.hpp
#pragma once

#include "esp_camera.h"

/**
 * @brief Resize an RGB565 image using bilinear interpolation.
 *
 * @param src_frame Source frame buffer (RGB565 format).
 * @param dst_width Desired width of the resized image.
 * @param dst_height Desired height of the resized image.
 * @return Pointer to the resized frame buffer. NULL if resizing fails.
 *
 * @note The caller is responsible for returning the resized frame using esp_camera_fb_return().
 */
camera_fb_t* resize_frame_bilinear(const camera_fb_t* src_frame, int dst_width, int dst_height);
//Filename: app_lcd/include/logo_en_240x240_lcd.h
#include <stdint.h>

static int logo_en_240x240_lcd_width = 240;
static int logo_en_240x240_lcd_height = 240;

//Filename: app_lcd/src/app_lcd.cpp
#include "app_lcd.hpp"
#include "image_utils.hpp"

#include <string.h>

#include "esp_log.h"
#include "esp_camera.h"

#include "logo_en_240x240_lcd.h"

static const char TAG[] = "App/LCD";

AppLCD::AppLCD(QueueHandle_t queue_i,
               QueueHandle_t queue_o,
               void (*callback)(camera_fb_t *)) : Frame(queue_i, queue_o, callback),
                                                  panel_handle(NULL),
                                                  switch_on(true)
{
    do
    {
        ESP_LOGI(TAG, "Initialize SPI bus");
        spi_bus_config_t bus_conf = {
            .mosi_io_num = BOARD_LCD_MOSI,
            .miso_io_num = BOARD_LCD_MISO,
            .sclk_io_num = BOARD_LCD_SCK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = BOARD_LCD_H_RES * BOARD_LCD_V_RES * sizeof(uint16_t),
        };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_conf, SPI_DMA_CH_AUTO));

        ESP_LOGI(TAG, "Install panel IO");
        esp_lcd_panel_io_handle_t io_handle = NULL;
        esp_lcd_panel_io_spi_config_t io_config = {
            .cs_gpio_num = BOARD_LCD_CS,
            .dc_gpio_num = BOARD_LCD_DC,
            .spi_mode = 0,
            .pclk_hz = BOARD_LCD_PIXEL_CLOCK_HZ,
            .trans_queue_depth = 10,
            .lcd_cmd_bits = BOARD_LCD_CMD_BITS,
            .lcd_param_bits = BOARD_LCD_PARAM_BITS,
        };
        // Attach the LCD to the SPI bus
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

        // ESP_LOGI(TAG, "Install ST7789 panel driver");
        esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = BOARD_LCD_RST,
            .rgb_endian = LCD_RGB_ENDIAN_RGB,
            .bits_per_pixel = 16,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
        esp_lcd_panel_invert_color(panel_handle, true);// Set inversion for esp32s3eye

        // turn on display
        esp_lcd_panel_disp_on_off(panel_handle, true);

        this->draw_color(0x000000);
        vTaskDelay(pdMS_TO_TICKS(500));
        this->draw_wallpaper();
        vTaskDelay(pdMS_TO_TICKS(1000));
    } while (0);
}

void AppLCD::draw_wallpaper()
{
    uint16_t *pixels = (uint16_t *)heap_caps_malloc((logo_en_240x240_lcd_width * logo_en_240x240_lcd_height) * sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (NULL == pixels)
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
        return;
    }
    memcpy(pixels, logo_en_240x240_lcd, (logo_en_240x240_lcd_width * logo_en_240x240_lcd_height) * sizeof(uint16_t));
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, logo_en_240x240_lcd_width, logo_en_240x240_lcd_height, (uint16_t *)pixels);
    heap_caps_free(pixels);

    this->paper_drawn = true;
}

void AppLCD::draw_color(int color)
{
    uint16_t *buffer = (uint16_t *)malloc(BOARD_LCD_H_RES * sizeof(uint16_t));
    if (NULL == buffer)
    {
        ESP_LOGE(TAG, "Memory for bitmap is not enough");
    }
    else
    {
        for (size_t i = 0; i < BOARD_LCD_H_RES; i++)
        {
            buffer[i] = color;
        }

        for (int y = 0; y < BOARD_LCD_V_RES; y++)
        {
            esp_lcd_panel_draw_bitmap(panel_handle, 0, y, BOARD_LCD_H_RES, y+1, buffer);
        }

        free(buffer);
    }
}

void AppLCD::update()
{
    if (this->switch_on == false)
    {
        this->paper_drawn = false;
    }
}

static void task(AppLCD *self)
{
    ESP_LOGI(TAG, "Start LCD Task");

    camera_fb_t *frame = nullptr;
    while (true)
    {
        if (self->queue_i == nullptr)
            break;

        if (xQueueReceive(self->queue_i, &frame, portMAX_DELAY))
        {
            if (self->switch_on) {
                camera_fb_t *resized_frame = resize_frame_bilinear(frame, 240, 240);
                if(resized_frame == nullptr) {
                    ESP_LOGE(TAG, "Failed to resize frame");
                    continue;
                }
                
                esp_lcd_panel_draw_bitmap(self->panel_handle, 0, 0, 240, 240, (uint16_t *)resized_frame->buf);
                ESP_LOGI(TAG, "Drew a frame");
            }
            else if (self->paper_drawn == false) {
                self->draw_wallpaper();
                ESP_LOGI(TAG, "Drew a wallpaper");
            }

            if (self->queue_o)
                xQueueSend(self->queue_o, &frame, portMAX_DELAY);
            else
                self->callback(frame);
        }
    }
    ESP_LOGD(TAG, "Stop LCD Task");
    self->draw_wallpaper();
    vTaskDelete(NULL);
}

void AppLCD::run()
{
    xTaskCreatePinnedToCore((TaskFunction_t)task, TAG, 4 * 1024, this, 5, NULL, 1);
}
//Filename: app_lcd/src/image_utils.cpp
// Filename: components/app_lcd/src/image_utils.cpp
#include "image_utils.hpp"
#include "esp_log.h"
#include <stdlib.h>
#include <math.h>

static const char* TAG = "ImageUtils";

/**
 * @brief Bilinear interpolation resizing for RGB565 images.
 *
 * @param src_frame Source frame buffer.
 * @param dst_width Desired width.
 * @param dst_height Desired height.
 * @return Resized frame buffer or NULL on failure.
 */
camera_fb_t* resize_frame_bilinear(const camera_fb_t* src_frame, int dst_width, int dst_height) {
    if (!src_frame) {
        ESP_LOGE(TAG, "Source frame is NULL");
        return NULL;
    }

    if (src_frame->format != PIXFORMAT_RGB565) {
        ESP_LOGE(TAG, "Unsupported pixel format. Expected RGB565");
        return NULL;
    }

    // Allocate memory for the resized frame
    camera_fb_t* resized_frame = (camera_fb_t*) src_frame;
    if (!resized_frame) {
        ESP_LOGE(TAG, "Failed to allocate memory for resized frame");
        return NULL;
    }

    resized_frame->width = dst_width;
    resized_frame->height = dst_height;
    resized_frame->format = src_frame->format;
    resized_frame->len = dst_width * dst_height * 2; // RGB565 uses 2 bytes per pixel

    const uint16_t* src_pixels = (const uint16_t*)src_frame->buf;
    uint16_t* dst_pixels = (uint16_t*)resized_frame->buf;

    float x_ratio = ((float)(src_frame->width - 1)) / dst_width;
    float y_ratio = ((float)(src_frame->height - 1)) / dst_height;
    float x_diff, y_diff;
    int x, y;
    uint16_t a, b, c, d;
    uint16_t pixel;

    for (int i = 0; i < dst_height; i++) {
        for (int j = 0; j < dst_width; j++) {
            x = (int)(x_ratio * j);
            y = (int)(y_ratio * i);
            x_diff = (x_ratio * j) - x;
            y_diff = (y_ratio * i) - y;

            // Get four neighboring pixels
            a = src_pixels[y * src_frame->width + x];
            b = src_pixels[y * src_frame->width + (x + 1)];
            c = src_pixels[(y + 1) * src_frame->width + x];
            d = src_pixels[(y + 1) * src_frame->width + (x + 1)];

            // Extract RGB components
            uint8_t a_r = (a & 0xF800) >> 11;
            uint8_t a_g = (a & 0x07E0) >> 5;
            uint8_t a_b = (a & 0x001F);

            uint8_t b_r = (b & 0xF800) >> 11;
            uint8_t b_g = (b & 0x07E0) >> 5;
            uint8_t b_b = (b & 0x001F);

            uint8_t c_r = (c & 0xF800) >> 11;
            uint8_t c_g = (c & 0x07E0) >> 5;
            uint8_t c_b = (c & 0x001F);

            uint8_t d_r = (d & 0xF800) >> 11;
            uint8_t d_g = (d & 0x07E0) >> 5;
            uint8_t d_b = (d & 0x001F);

            // Perform bilinear interpolation for each color channel
            float red = a_r * (1 - x_diff) * (1 - y_diff) +
                        b_r * x_diff * (1 - y_diff) +
                        c_r * y_diff * (1 - x_diff) +
                        d_r * x_diff * y_diff;

            float green = a_g * (1 - x_diff) * (1 - y_diff) +
                          b_g * x_diff * (1 - y_diff) +
                          c_g * y_diff * (1 - x_diff) +
                          d_g * x_diff * y_diff;

            float blue = a_b * (1 - x_diff) * (1 - y_diff) +
                         b_b * x_diff * (1 - y_diff) +
                         c_b * y_diff * (1 - x_diff) +
                         d_b * x_diff * y_diff;

            // Combine channels back into RGB565
            uint16_t r = (uint16_t)(red) & 0x1F;
            uint16_t g = ((uint16_t)(green) & 0x3F) << 5;
            uint16_t b_final = ((uint16_t)(blue) & 0x1F);

            pixel = r | g | b_final;

            dst_pixels[i * dst_width + j] = pixel;
        }
    }

    return resized_frame;
}
//Filename: ../main/app_main.cpp
#include "esp_log.h"
// #include "jpeg_decoder.h"
#include "pedestrian_detect.hpp"

#include "driver/gpio.h"

#include "app_camera.hpp"
#include "app_lcd.hpp"
#include "app_detect.hpp"

// extern const uint8_t pedestrian_jpg_start[] asm("_binary_pedestrian_jpg_start");
// extern const uint8_t pedestrian_jpg_end[] asm("_binary_pedestrian_jpg_end");
const char *TAG = "MAIN"; 

// uint8_t *get_image(const uint8_t *jpg_img, uint32_t jpg_img_size, int height, int width)
// {
//     uint32_t outbuf_size = height * width * 3;
//     uint8_t *outbuf = (uint8_t *)heap_caps_malloc(outbuf_size, MALLOC_CAP_SPIRAM);
//     // JPEG decode config
//     esp_jpeg_image_cfg_t jpeg_cfg = {.indata = (uint8_t *)jpg_img,
//                                      .indata_size = jpg_img_size,
//                                      .outbuf = outbuf,
//                                      .outbuf_size = outbuf_size,
//                                      .out_format = JPEG_IMAGE_FORMAT_RGB888,
//                                      .out_scale = JPEG_IMAGE_SCALE_0,
//                                      .flags = {
//                                          .swap_color_bytes = 1,
//                                      }};

//     esp_jpeg_image_output_t outimg;
//     esp_jpeg_decode(&jpeg_cfg, &outimg);
//     assert(outimg.height == height && outimg.width == width);
//     return outbuf;
// }

extern "C" void app_main(void)
{
    QueueHandle_t xQueueFrame_0 = xQueueCreate(5, sizeof(camera_fb_t *));
    QueueHandle_t xQueueFrame_1 = xQueueCreate(5, sizeof(camera_fb_t *));

    // AppCamera *camera = new AppCamera(PIXFORMAT_RGB565, FRAMESIZE_240X240, 2, xQueueFrame_0);
    AppCamera *camera = new AppCamera(PIXFORMAT_JPEG, FRAMESIZE_VGA, 2, xQueueFrame_0);
    AppDetect *detector = new AppDetect(xQueueFrame_0, xQueueFrame_1);
    // AppLCD *lcd = new AppLCD(xQueueFrame_1);

    camera->run();
    // lcd->run();
    detector->run();
}
