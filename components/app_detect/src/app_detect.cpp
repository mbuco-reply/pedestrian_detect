#include "app_detect.hpp"
#include "../../common.hpp"
#include "jpeg_decoder.h"
#include "esp_camera.h"

static const char TAG[] = "App/Detect";

void draw_detection_result(uint8_t *image_ptr, int image_height, int image_width, std::list<dl::detect::result_t> &results)
{
    int i = 0;
    for (std::list<dl::detect::result_t>::iterator prediction = results.begin(); prediction != results.end(); prediction++, i++)
    {
        dl::image::draw_hollow_rectangle(image_ptr, image_height, image_width,
                                         DL_MAX(prediction->box[0], 0),
                                         DL_MAX(prediction->box[1], 0),
                                         DL_MAX(prediction->box[2], 0),
                                         DL_MAX(prediction->box[3], 0),
                                         0x00FF00);
    }
}

res_detect_t prepare_results_for_transfer(std::list<dl::detect::result_t> &results)
{
    res_detect_t res;
    res.count = results.size();
    int i = 0;

    for (std::list<dl::detect::result_t>::iterator prediction = results.begin(); prediction != results.end(); prediction++, i++)
    {
        res_rect_t rect = {
            .corners = {
                (uint16_t)prediction->box[0],
                (uint16_t)prediction->box[1],
                (uint16_t)prediction->box[2],
                (uint16_t)prediction->box[3]
            }
        };
        res.results[i] = rect;           
    }

    return res;
}

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
    self->detect = new PedestrianDetect();

    // Variables for FPS counting
    static int frame_count = 0;
    static int64_t last_fps_time = esp_timer_get_time(); // in microseconds

    while (true)
    {
        if (self->queue_i == nullptr)
            break;

        if (xQueueReceive(self->queue_i, &frame, portMAX_DELAY) == pdTRUE)
        {
            // Increment frame_count for FPS calculation
            frame_count++;

            res_detect_t res;
            uint8_t *img = get_image(frame->buf, frame->len, frame->height, frame->width);
            esp_camera_fb_return(frame);

            auto &detect_results = self->detect->run((uint8_t *)img, {FRAME_HEIGHT, FRAME_WIDTH, 3});

            if (detect_results.size())
            {
                ESP_LOGI(TAG,"Detected %d objects", (int)detect_results.size());
                res = prepare_results_for_transfer(detect_results);
                
                print_detected_result(detect_results);
                // draw_detection_result((uint8_t *)img, 480, 640, detect_results);
            }
            else {
                ESP_LOGE(TAG, "No object detected");
            }

            if (self->queue_o) {
                res_frame_t send_frame = {
                    .buffer = (uint8_t *)img,
                    .len = FRAME_HEIGHT * FRAME_WIDTH * 3,
                    .detect_results = res
                };
                xQueueSend(self->queue_o, &send_frame, portMAX_DELAY);
            }

            // FPS calculation: Check if one second has passed
            int64_t current_time = esp_timer_get_time(); // in microseconds
            if ((current_time - last_fps_time) >= 1000000) {
                // One second or more has elapsed
                float fps = (float)frame_count * 1000000.0f / (float)(current_time - last_fps_time);
                ESP_LOGI(TAG, "Current FPS: %.2f", fps);

                // Reset for next measurement
                frame_count = 0;
                last_fps_time = current_time;
            }
        }
        else {
            ESP_LOGE(TAG, "Failed to receive frame");
        }
    }

    ESP_LOGD(TAG, "Stop");
    vTaskDelete(NULL);
}


void AppDetect::run()
{
    xTaskCreatePinnedToCore((TaskFunction_t)task, TAG, 5 * 1024, this, 5, NULL, 1);
}
