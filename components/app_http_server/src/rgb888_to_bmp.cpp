// File: components/app_rgb888_to_bmp/rgb888_to_bmp.cpp
#include "rgb888_to_bmp.hpp"
#include <cstdint>
#include <cstring>
#include <cstdlib>

bool rgb888_to_bmp(const uint8_t* rgb_buffer, int width, int height, uint8_t** bmp_buffer, size_t* bmp_size) {
    // Calculate padding per row (each row must be a multiple of 4 bytes)
    int row_stride = width * 3;
    int padding = (4 - (row_stride) % 4) % 4;
    int padded_row_stride = row_stride + padding;

    // Calculate total BMP size
    *bmp_size = 14 + 40 + (padded_row_stride * height); // FileHeader + InfoHeader + Pixel Data

    // Allocate memory for BMP
    *bmp_buffer = (uint8_t*)malloc(*bmp_size);
    if (*bmp_buffer == NULL) {
        return false;
    }

    memset(*bmp_buffer, 0, *bmp_size); // Initialize buffer to zero

    // ----------------- Bitmap File Header (14 bytes) -----------------
    // Signature 'BM'
    (*bmp_buffer)[0] = 'B';
    (*bmp_buffer)[1] = 'M';

    // File size (4 bytes, little endian)
    uint32_t file_size = (uint32_t)(*bmp_size);
    memcpy(&(*bmp_buffer)[2], &file_size, 4);

    // Reserved (4 bytes) - already zero

    // Data offset (4 bytes, little endian) - 14 (file header) + 40 (info header) = 54
    uint32_t data_offset = 54;
    memcpy(&(*bmp_buffer)[10], &data_offset, 4);

    // ----------------- DIB Header (BITMAPINFOHEADER - 40 bytes) -----------------
    // Header size (4 bytes, little endian)
    uint32_t header_size = 40;
    memcpy(&(*bmp_buffer)[14], &header_size, 4);

    // Image width (4 bytes, little endian)
    int32_t img_width = width;
    memcpy(&(*bmp_buffer)[18], &img_width, 4);

    // Image height (4 bytes, little endian)
    int32_t img_height = height;
    memcpy(&(*bmp_buffer)[22], &img_height, 4);

    // Color planes (2 bytes, little endian) - must be 1
    uint16_t planes = 1;
    memcpy(&(*bmp_buffer)[26], &planes, 2);

    // Bits per pixel (2 bytes, little endian) - 24 for RGB888
    uint16_t bpp = 24;
    memcpy(&(*bmp_buffer)[28], &bpp, 2);

    // Compression (4 bytes, little endian) - 0 (BI_RGB, no compression)
    uint32_t compression = 0;
    memcpy(&(*bmp_buffer)[30], &compression, 4);

    // Image size (4 bytes, little endian) - size of raw bitmap data (including padding)
    uint32_t img_size = padded_row_stride * height;
    memcpy(&(*bmp_buffer)[34], &img_size, 4);

    // Horizontal resolution (4 bytes, little endian) - pixels per meter (e.g., 2835 for 72 DPI)
    uint32_t h_resolution = 2835;
    memcpy(&(*bmp_buffer)[38], &h_resolution, 4);

    // Vertical resolution (4 bytes, little endian) - pixels per meter
    uint32_t v_resolution = 2835;
    memcpy(&(*bmp_buffer)[42], &v_resolution, 4);

    // Colors in palette (4 bytes, little endian) - 0 for default
    uint32_t colors_in_palette = 0;
    memcpy(&(*bmp_buffer)[46], &colors_in_palette, 4);

    // Important colors (4 bytes, little endian) - 0 for all
    uint32_t important_colors = 0;
    memcpy(&(*bmp_buffer)[50], &important_colors, 4);

    // ----------------- Pixel Data -----------------
    // BMP stores pixels in BGR format and starts from the bottom-left corner
    uint8_t* pixel_data = &(*bmp_buffer)[54];
    for (int y = 0; y < height; y++) {
        // Pointer to the current row in the RGB buffer
        const uint8_t* row_ptr = rgb_buffer + (y * width * 3);
        // Pointer to the current row in the BMP buffer (bottom-up)
        uint8_t* bmp_row_ptr = pixel_data + ((height - 1 - y) * padded_row_stride);

        for (int x = 0; x < width; x++) {
            // BMP uses BGR format
            bmp_row_ptr[x * 3 + 0] = row_ptr[x * 3 + 2]; // B
            bmp_row_ptr[x * 3 + 1] = row_ptr[x * 3 + 1]; // G
            bmp_row_ptr[x * 3 + 2] = row_ptr[x * 3 + 0]; // R
        }

        // Padding bytes are already zeroed by memset
    }

    return true;
}
