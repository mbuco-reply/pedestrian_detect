// File: components/app_rgb888_to_bmp/rgb888_to_bmp.cpp
#include "rgb888_to_bmp.hpp"

bool rgb888_to_bmp(uint8_t* rgb_buffer, int src_width, int src_height,
                   int dst_width, int dst_height,
                   uint8_t** bmp_buffer, size_t* bmp_size) {

    // Calculate padding per row for the output image
    int row_stride = dst_width * 3;
    int padding = (4 - (row_stride % 4)) % 4;
    int padded_row_stride = row_stride + padding;

    // Calculate total BMP size
    *bmp_size = 14 + 40 + (padded_row_stride * dst_height);

    // Allocate memory for BMP
    *bmp_buffer = (uint8_t*)malloc(*bmp_size);
    if (*bmp_buffer == NULL) {
        return false;
    }

    // Initialize headers
    uint8_t* buf = *bmp_buffer;
    memset(buf, 0, *bmp_size); // Ensure padding areas are zeroed

    // File header
    buf[0] = 'B'; buf[1] = 'M';
    uint32_t file_size = (uint32_t)(*bmp_size);
    memcpy(&buf[2], &file_size, 4);
    uint32_t data_offset = 54;
    memcpy(&buf[10], &data_offset, 4);

    // DIB header
    uint32_t header_size = 40;
    memcpy(&buf[14], &header_size, 4);
    int32_t img_width = dst_width;
    int32_t img_height = dst_height;
    memcpy(&buf[18], &img_width, 4);
    memcpy(&buf[22], &img_height, 4);
    uint16_t planes = 1;
    memcpy(&buf[26], &planes, 2);
    uint16_t bpp = 24;
    memcpy(&buf[28], &bpp, 2);
    uint32_t compression = 0;
    memcpy(&buf[30], &compression, 4);
    uint32_t img_size = padded_row_stride * dst_height;
    memcpy(&buf[34], &img_size, 4);
    uint32_t resolution = 2835; // 72 DPI
    memcpy(&buf[38], &resolution, 4);
    memcpy(&buf[42], &resolution, 4);
    uint32_t colors_in_palette = 0;
    memcpy(&buf[46], &colors_in_palette, 4);
    uint32_t important_colors = 0;
    memcpy(&buf[50], &important_colors, 4);

    // Pixel data
    uint8_t* pixel_data = &buf[54];

    // Downsample and convert RGB888 to BGR in one step
    for (int y = 0; y < dst_height; y++) {
        int src_y = (y * src_height) / dst_height;
        uint8_t* bmp_row_ptr = pixel_data + ((dst_height - 1 - y) * padded_row_stride);

        for (int x = 0; x < dst_width; x++) {
            int src_x = (x * src_width) / dst_width;

            const uint8_t* src_pixel = rgb_buffer + (src_y * src_width * 3) + (src_x * 3);
            // BGR output
            bmp_row_ptr[x * 3 + 0] = src_pixel[2]; // B
            bmp_row_ptr[x * 3 + 1] = src_pixel[1]; // G
            bmp_row_ptr[x * 3 + 2] = src_pixel[0]; // R
        }
        // Padding is already zeroed by memset
    }

    return true;
}

/**
 * @brief Draws green rectangles onto a BMP buffer based on detection results.
 *
 * @param bmp_buffer    Pointer to the BMP data (including headers) returned by rgb888_to_bmp.
 * @param bmp_size      Size of the BMP buffer in bytes.
 * @param dst_width     Width of the downscaled image.
 * @param dst_height    Height of the downscaled image.
 * @param src_width     Original width of the image before downscaling.
 * @param src_height    Original height of the image before downscaling.
 * @param detect_results Pointer to a res_detect_t that contains the count and rectangles to be drawn.
 */
void draw_detected_rectangles_on_bmp(
    uint8_t* bmp_buffer,
    size_t bmp_size,
    int dst_width,
    int dst_height,
    int src_width,
    int src_height,
    res_detect_t *detect_results)
{
    // Calculate strides and offsets for the BMP
    // Each pixel is 3 bytes (BGR), and we may have padding per line.
    int row_stride = dst_width * 3;
    int padding = (4 - (row_stride % 4)) % 4;
    int padded_row_stride = row_stride + padding;

    // Pixel data starts at offset 54 in the BMP buffer
    uint8_t* pixel_data = bmp_buffer + 54;

    // Define rectangle color: green in BGR is (0,255,0)
    uint8_t B = 0;
    uint8_t G = 255;
    uint8_t R = 0;

    // Lambda function to draw a single pixel in the BMP buffer at (x, y)
    auto draw_pixel = [&](int x, int y) {
        if (x < 0 || x >= dst_width || y < 0 || y >= dst_height) return;
        int bmp_row = (dst_height - 1 - y);
        uint8_t* pixel_ptr = pixel_data + (bmp_row * padded_row_stride) + (x * 3);
        pixel_ptr[0] = B;
        pixel_ptr[1] = G;
        pixel_ptr[2] = R;
    };

    // Iterate over all detected rectangles
    for (int i = 0; i < detect_results->count; i++) {
        const res_rect_t &rect = detect_results->results[i];
        // rect.corners = [x1, y1, x2, y2]
        int left_up_x = rect.corners[0];
        int left_up_y = rect.corners[1];
        int right_down_x = rect.corners[2];
        int right_down_y = rect.corners[3];

        // Scale coordinates from original to downscaled dimensions
        int scaled_x1 = (left_up_x * dst_width) / src_width;
        int scaled_y1 = (left_up_y * dst_height) / src_height;
        int scaled_x2 = (right_down_x * dst_width) / src_width;
        int scaled_y2 = (right_down_y * dst_height) / src_height;

        // Clamp coordinates to ensure they are within the image bounds
        if (scaled_x1 < 0) scaled_x1 = 0;
        if (scaled_y1 < 0) scaled_y1 = 0;
        if (scaled_x2 >= dst_width) scaled_x2 = dst_width - 1;
        if (scaled_y2 >= dst_height) scaled_y2 = dst_height - 1;

        // Draw the rectangle edges:
        // Top edge: y = scaled_y1, from x=scaled_x1 to x=scaled_x2
        for (int x = scaled_x1; x <= scaled_x2; x++) {
            draw_pixel(x, scaled_y1);
        }

        // Bottom edge: y = scaled_y2, from x=scaled_x1 to x=scaled_x2
        for (int x = scaled_x1; x <= scaled_x2; x++) {
            draw_pixel(x, scaled_y2);
        }

        // Left edge: x = scaled_x1, from y=scaled_y1 to y=scaled_y2
        for (int y = scaled_y1; y <= scaled_y2; y++) {
            draw_pixel(scaled_x1, y);
        }

        // Right edge: x = scaled_x2, from y=scaled_y1 to y=scaled_y2
        for (int y = scaled_y1; y <= scaled_y2; y++) {
            draw_pixel(scaled_x2, y);
        }
    }
}
