#ifndef PREDEFINE_MACRO_H
#define PREDEFINE_MACRO_H

// #ifndef DEBUG
// #define DEBUG
// #endif

// #ifndef MAKEMD
// #define MAKEMD
// #endif

#ifndef RUNGT
#define RUNGT
#endif

//for  NearNeighborDisMetric
//track保存之前帧数（特征）的阈值
#define args_nn_budget 100
//特征余弦距离距离阈值
//#define args_max_cosine_distance 0.2  //MOT16
#define args_max_cosine_distance 0.2 //OSNet


//for Mot 16 datasets
// //检测框的置信度得分阈值，置信度得分低于阈值的框应该被忽略
// #define args_min_confidence 0.3
// //进行NMS时，相似检测框的iou阈值
// #define args_nms_max_overlap 1.0

//for ModelDetection(RFSong-7993)
//检测框的置信度得分阈值，置信度得分低于阈值的框应该被忽略
#define args_min_confidence 0.02
//进行NMS时，相似检测框的iou阈值
#define args_nms_max_overlap 0.4
 //进行NMS时，根据置信度保留的最大检测框数。
#define args_top_k 5000
//进行NMS后，每个图像要保留的总检测框数。
#define args_keep_top_k 750
//显示预测框时的得分阈值，小于该阈值的将不被显示
#define args_vis_thres 0.3

//for MOT imgs & .dat
//download imgs:    https://motchallenge.net/data/MOT16/
//download npy/dat:     https://drive.google.com/open?id=18fKzfqnqhqW3s9zwsCbnVJ5XF2JFeqMp
// #define DATDIR "/home/liujie/code/deep_sort/resources/detections/MOT16_POI_test/"
// #define MOTDIR "/home/liujie/code/deep_sort/MOT16/test/MOT16-06/"
//#define VIDEO "./RUNNINGDATA/test.avi"

//RFB CONFIGS
//VOC_Config
const float prior_variance[]={0.1, 0.2};
#endif