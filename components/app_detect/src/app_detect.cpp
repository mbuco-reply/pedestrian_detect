#include "app_detect.hpp"

#include <list>
#include "esp_log.h"
#include "esp_camera.h"

#include "dl_image.hpp"
#include "fb_gfx.h"
#include "who_ai_utils.hpp"
#include "jpeg_decoder.h"

static const char TAG[] = "App/Detect";

#define RGB565_MASK_RED 0xF800
#define RGB565_MASK_GREEN 0x07E0
#define RGB565_MASK_BLUE 0x001F

// static void rgb_print(camera_fb_t *fb, uint32_t color, const char *str)
// {
//     fb_gfx_print(fb, (fb->width - (strlen(str) * 14)) / 2, 10, color, str);
// }

// static int rgb_printf(camera_fb_t *fb, uint32_t color, const char *format, ...)
// {
//     char loc_buf[64];
//     char *temp = loc_buf;
//     int len;
//     va_list arg;
//     va_list copy;
//     va_start(arg, format);
//     va_copy(copy, arg);
//     len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
//     va_end(copy);
//     if (len >= sizeof(loc_buf))
//     {
//         temp = (char *)malloc(len + 1);
//         if (temp == NULL)
//         {
//             return 0;
//         }
//     }
//     vsnprintf(temp, len + 1, format, arg);
//     va_end(arg);
//     rgb_print(fb, color, temp);
//     if (len > 64)
//     {
//         free(temp);
//     }
//     return len;
// }


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

static camera_fb_t img_frame = {
    .len = 640 * 480 * 3,
    .width = 640,
    .height = 480,
    .format = PIXFORMAT_RGB888
};

camera_fb_t * get_img_cam(camera_fb_t * frame)
{
    img_frame.buf = get_image(frame->buf, 640*480*3, 480, 640);
    return &img_frame;
}

void print_detected_result(std::list<dl::detect::result_t> &detect_results)
{
    for (const auto &res : detect_results) {
        ESP_LOGI(TAG,
                    "[score: %f, x1: %d, y1: %d, x2: %d, y2: %d]\n",
                    res.score,
                    res.box[0],
                    res.box[1],
                    res.box[2],
                    res.box[3]);
    }
}

AppDetect::AppDetect(QueueHandle_t queue_i,
                 QueueHandle_t queue_o) : Frame(queue_i, queue_o)
{}

AppDetect::~AppDetect()
{}

static void task(AppDetect *self)
{
    ESP_LOGD(TAG, "Start Human Detection");
    camera_fb_t *frame = nullptr;
    // PedestrianDetect *detect = new PedestrianDetect();
    self->detect = new PedestrianDetect();

    while (true)
    {
        if (self->queue_i == nullptr)
            break;

        if (xQueueReceive(self->queue_i, &frame, portMAX_DELAY) == pdTRUE)
        {
            // ESP_LOGI(TAG, "Got a frame");
            uint8_t *img = get_image(frame->buf, frame->len, frame->height, frame->width);
            esp_camera_fb_return(frame);

            auto &detect_results = self->detect->run((uint8_t *)img, {480, 640, 3});
            // std::list<dl::detect::result_t> &detect_results = self->detector.infer((uint16_t *)frame->buf, {(int)frame->height, (int)frame->width, 3});

            free(img);

            if (detect_results.size())
            {
                ESP_LOGI(TAG,"Detected %d objects", detect_results.size());
                print_detected_result(detect_results);
                // draw_detection_result((uint16_t *)frame->buf, frame->height, frame->width, detect_results);
                // rgb_printf(frame, RGB565_MASK_RED, "%d IDs left", self->recognizer->get_enrolled_id_num());
            }
            else {
                ESP_LOGE(TAG, "No object detected");
            }

            // if (self->queue_o) {
            //     // ESP_LOGI(TAG, "Passing frame to output queue");
            //     xQueueSend(self->queue_o, &frame, portMAX_DELAY);
            // }
        }
        else {
            ESP_LOGE(TAG, "Failed to receive frame");
        }
        
    }
    
    ESP_LOGD(TAG, "Stop");
    free(frame);
    vTaskDelete(NULL);
}

void AppDetect::run()
{
    xTaskCreatePinnedToCore((TaskFunction_t)task, TAG, 5 * 1024, this, 5, NULL, 1);
}
