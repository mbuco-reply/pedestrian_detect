#include "app_camera.hpp"

#include "esp_log.h"
#include "esp_system.h"
#include "jpeg_decoder.h"

const static char TAG[] = "App/Camera";

extern const uint8_t pedestrian_jpeg_start[] asm("_binary_pedestrian_jpeg_start");
extern const uint8_t pedestrian_jpeg_end[] asm("_binary_pedestrian_jpeg_end");

// static camera_fb_t img_frame = {
//     .len = 640 * 480 * 3,
//     .width = 640,
//     .height = 480,
//     .format = PIXFORMAT_JPEG
// };

// static camera_fb_t img_frame_pic = {
//     .len = (size_t)(pedestrian_jpeg_end - pedestrian_jpeg_start), // Size of the JPEG data
//     .width = 640,
//     .height = 480,
//     .format = PIXFORMAT_JPEG
// };

// camera_fb_t * get_img()
// {
//     img_frame_pic.buf = (uint8_t *)pedestrian_jpeg_start; // Point directly to JPEG data
//     img_frame_pic.len = pedestrian_jpeg_end - pedestrian_jpeg_start;
//     img_frame_pic.format = PIXFORMAT_JPEG; // Ensure format is JPEG
//     return &img_frame_pic;
// }

static camera_fb_t img_frame = {
    .len = 640 * 480 * 3,
    .width = 640,
    .height = 480,
    .format = PIXFORMAT_RGB888
};


uint8_t *get_image(const uint8_t *jpg_img, uint32_t jpg_img_size, int height, int width)
{
    uint32_t outbuf_size = height * width * 3;
    uint8_t *outbuf = (uint8_t *)heap_caps_malloc(outbuf_size, MALLOC_CAP_SPIRAM);
    // JPEG decode config
    esp_jpeg_image_cfg_t jpeg_cfg = {.indata = (uint8_t *)jpg_img,
                                     .indata_size = jpg_img_size,
                                     .outbuf = outbuf,
                                     .outbuf_size = outbuf_size,
                                     .out_format = JPEG_IMAGE_FORMAT_RGB888,
                                     .out_scale = JPEG_IMAGE_SCALE_0,
                                     .flags = {
                                         .swap_color_bytes = 1,
                                     }};

    esp_jpeg_image_output_t outimg;
    esp_jpeg_decode(&jpeg_cfg, &outimg);
    assert(outimg.height == height && outimg.width == width);
    return outbuf;
}

camera_fb_t * get_img()
{
    img_frame.buf = get_image(pedestrian_jpeg_start, pedestrian_jpeg_end - pedestrian_jpeg_start, 480, 640);
    return &img_frame;
}

camera_fb_t * get_img_cam(camera_fb_t * frame)
{
    img_frame.buf = get_image(frame->buf, 640*480*3, 480, 640);
    return &img_frame;
}

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

        camera_fb_t *frame_cam = esp_camera_fb_get();
        // camera_fb_t *frame = get_img();
        camera_fb_t *frame = get_img_cam(frame_cam);
        
        if (frame) {
            // ESP_LOGI(TAG, "Received frame from camera, passing it to output queue");
            xQueueSend(self->queue_o, &frame, portMAX_DELAY);
        }

        if(frame) {
            ESP_LOGI(TAG, "Frame received is of dimensions %dx%d", frame->width, frame->height);
        }

        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }

    ESP_LOGD(TAG, "Stop");
    vTaskDelete(NULL);
}

void AppCamera::run()
{
    xTaskCreatePinnedToCore((TaskFunction_t)task, TAG, 6 * 1024, this, 5, NULL, 0);
}
