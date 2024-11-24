#pragma once

#include "sdkconfig.h"

#include "__base__.hpp"
#include "app_camera.hpp"
#include "pedestrian_detect.hpp"

class AppDetect : public Frame
{
private:

public:
    PedestrianDetect *detect;
    uint8_t frame_count = 0;

    AppDetect(QueueHandle_t queue_i = nullptr,
            QueueHandle_t queue_o = nullptr);
    ~AppDetect();

    void run();
};
