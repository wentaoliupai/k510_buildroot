#!/bin/sh

devmem 0x970E00fc 32 0x0fffff00
devmem 0x970E0100 32 0x000000ff
devmem 0x970E00f4 32 0x00550000

cd ../exe && ./face_landmarks ../kmodel/kmodel_release/face_detect/mb_rf320/retinaface_mobile0.25_320_bf16_with_preprocess.kmodel 320 320 240 0.95 0.2 ../kmodel/kmodel_release/face_landmarks/pfld_106/v2/v2_process_bf16_with_preprocess.kmodel   112 106 0.00001 2 ./video_object_detect_320.conf  1 0 None
# cd ../exe && ./face_landmarks ../kmodel/kmodel_release/face_detect/mb_rf320/retinaface_mobile0.25_320_bf16_with_preprocess.kmodel 320 320 240 0.95 0.2 ../kmodel/kmodel_release/face_landmarks/pfld_106/v3/v3_process_bf16_with_preprocess.kmodel   112 106 0.00001 2 ./video_object_detect_320.conf  1 0 None