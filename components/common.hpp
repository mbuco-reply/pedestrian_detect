#pragma once

#include <list>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cstddef>

#include "esp_log.h"
#include "dl_detect_define.hpp"
#include "dl_image.hpp"
#include "esp_timer.h"

#define FRAME_WIDTH  640
#define FRAME_HEIGHT 480
#define FRAME_WIDTH_OUT 320
#define FRAME_HEIGHT_OUT 240

typedef struct {
    uint16_t corners[4];
} res_rect_t;

typedef struct {
    int count;
    res_rect_t results[10];
} res_detect_t;

// Structure to hold frame buffer information
typedef struct {
    uint8_t *buffer; // Pointer to the frame data (e.g., JPEG)
    size_t len;      // Length of the frame data
    res_detect_t detect_results; // Corners of the detected objects 
} res_frame_t;
