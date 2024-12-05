// File: components/app_rgb888_to_bmp/include/rgb888_to_bmp.hpp
#pragma once

#include "../../common.hpp"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Converts an RGB888 buffer to BMP format.
 *
 * @param rgb_buffer Pointer to the input RGB888 data.
 * @param width Width of the image.
 * @param height Height of the image.
 * @param bmp_buffer Pointer to the output buffer where BMP data will be stored.
 *                   The buffer is dynamically allocated inside the function.
 * @param bmp_size Pointer to a size_t variable to store the size of the BMP data.
 * @return true on success, false on failure.
 *
 * @note The caller is responsible for freeing the bmp_buffer using free().
 */
bool rgb888_to_bmp(uint8_t* rgb_buffer, int src_width, int src_height, int dst_width, int dst_height, uint8_t** bmp_buffer, size_t* bmp_size);
void draw_detected_rectangles_on_bmp(uint8_t* bmp_buffer, size_t bmp_size, int dst_width, int dst_height, int src_width, int src_height, res_detect_t *detect_results);

#ifdef __cplusplus
}
#endif
