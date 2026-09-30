# Delivery Box firmware — ESP32-S3-WROOM-1 N16R8
# Build:  idf.py build      Flash + monitor:  idf.py -p /dev/serial/by-id/usb-1a86_* flash monitor
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(delivery-box-esp32s3-microros)
