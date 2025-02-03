# Pedestrian Detection Example

| **Supported Targets** | ESP32-S3 | ESP32-P4 |
|-----------------------|----------|----------|

This project enables real-time pedestrian detection on ESP32 devices, capturing and processing camera frames and streaming the results via an HTTP server.

---

## Features
- Real-time pedestrian detection with bounding boxes.
- Live camera stream accessible through an HTTP server.
- Wi-Fi-enabled connectivity for remote monitoring.
- Approx. 3 FPS processing speed.

---

## Setup

### Requirements
- ESP-IDF installed on your system (compatible with ESP32-S3 or ESP32-P4).

### Installation Steps

1. Clone the ESP-IDF repository:
'''bash
   git clone https://github.com/espressif/esp-idf.git
'''
2. Clone this project repository:
'''bash
   git clone git@github.com:mbuco-reply/pedestrian_detect.git
'''
3. Install ESP-IDF dependencies:
   ./esp-idf/install.sh

4. Export the ESP-IDF environment:
   source ./esp-idf/export.sh

5. Navigate to the project directory:
   cd pedestrian_detect

6. Set the target (if necessary):
   export IDF_TARGET=esp32s3

7. Build, flash, and monitor the project:
   idf.py build flash monitor

---

## Running the Project

1. **Monitor the Terminal Output:**  
   Once the device is running, check the terminal for the line that shows the device's IP address:
   I (2443) esp_netif_handlers: sta ip: 192.168.178.39

2. **Access the HTTP Server:**  
   Open your browser and navigate to the displayed IP address, e.g., http://192.168.178.39.

---

## Example Output

The detection output should appear in the terminal after flashing and starting the project:

I (2136) pedestrian_detect: [score: 0.883883, x1: 283, y1: 191, x2: 371, y2: 462]  
I (2136) pedestrian_detect: [score: 0.870524, x1: 146, y1: 183, x2: 249, y2: 464]  
I (2136) pedestrian_detect: [score: 0.755190, x1: 411, y1: 226, x2: 487, y2: 392]

---

## Project Structure

pedestrian_detect  
├── CMakeLists.txt  
├── components  
│   ├── app_camera  
│   ├── app_detect  
│   ├── app_http_server  
│   ├── app_lcd  
│   └── app_wifi  
└── main  
    └── app_main.cpp  

---

## System Information

| **Category**         | **Detail**                                                                                     |
|----------------------|-------------------------------------------------------------------------------------------------|
| **Project Name**     | pedestrian_detect                                                                               |
| **Chip Model**       | ESP32-S3                                                                                        |
| **Flash Size**       | 8 MB                                                                                            |
| **PSRAM**            | 8 MB, 80 MHz Speed                                                                              |
| **Camera Detected**  | OV2640, Address: 0x30, Camera PID: 0x26                                                         |
| **Wi-Fi Connection** | Connected to SSID: BucoNexusHQ, RSSI: -47 dBm                                                   |
| **HTTP Server**      | Running on port 80, Endpoints: `/`, `/stream`, `/count`                                         |

---

## License

[MIT](LICENSE)
