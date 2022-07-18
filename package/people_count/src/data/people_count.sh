#!/bin/sh

devmem 0x970E00fc 32 0x0fffff00

./people_count ../kmodel/yolov5s_320_sigmoid_bf16_with_preprocess_uint8_NHWC.kmodel 320 320 240 0.5 0.45 video_320x320_240.conf 1 0 None 0.7 3 1000
