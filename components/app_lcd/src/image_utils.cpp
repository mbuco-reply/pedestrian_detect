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
