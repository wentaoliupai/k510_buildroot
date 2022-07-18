#ifndef PREDEFINE_MACRO_H
#define PREDEFINE_MACRO_H

// #ifndef DEBUG
// #define DEBUG
// #endif

//构造deep_sort.md文档所需输出
// #ifndef MAKEMD
// #define MAKEMD
// #endif

//const int FEATURE_LEN=128;      //for MOT16
const int  FEATURE_LEN=512;      //for OSNet

//for  NearNeighborDisMetric
//track保存之前帧数（特征）的阈值
const int MAX_NN_BUDGET=100;
//特征余弦距离距离阈值
const float MAX_COSINE_DISTANCE=0.2; //OSNet
//iou距离
const float MAX_IOU_DISTANCE=0.7;
//级联匹配的深度
const int MAX_AGE=30;
//当处于Tentativetrack状态的track,连续3帧与检测框匹配，则将track状态更改为Confirmed
const int MAX_N_INIT=3;
//track存在的最大帧数
const int MAX_HITS = 1000;


// //for Mot 16 datasets
// //检测框的置信度得分阈值，置信度得分低于阈值的框应该被忽略
// const float MIN_CONFIDENCE=0.3;
// //进行NMS时，相似检测框的iou阈值
// const float MAX_NMS_OVERLAP=1.0;


//for ModelDetection(RFSong-7993)
//检测框的置信度得分阈值，置信度得分低于阈值的框应该被忽略
const float MIN_CONFIDENCE=0.02;
//进行NMS时，相似检测框的iou阈值
const float MAX_NMS_OVERLAP=0.4;
 //进行NMS时，根据置信度保留的最大检测框数。
const int TOP_K=5000;
//进行NMS后，每个图像要保留的总检测框数。
const int KEEP_TOP_K=750;
//显示预测框时的得分阈值，小于该阈值的将不被显示
const float VISUAL_THRESHOLD=0.3;

//for ModelDetection(Yolo_v5)
//检测框的置信度得分阈值，置信度得分低于阈值的框应该被忽略
const float OBJ_THRESH_YOLOV5 = 0.5;
//进行NMS时，相似检测框的iou阈值
const float NMS_THRESH_YOLOV5 = 0.45;

#endif