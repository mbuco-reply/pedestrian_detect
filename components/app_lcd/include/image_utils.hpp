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
