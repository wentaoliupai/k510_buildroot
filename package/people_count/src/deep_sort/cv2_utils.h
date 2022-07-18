#ifndef _CV2_UTILS
#define _CV2_UTILS

#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include <definition/data_type.h>

#define SHARE_MEMORY_ALLOC          _IOWR('m', 1, unsigned long)
#define SHARE_MEMORY_ALIGN_ALLOC    _IOWR('m', 2, unsigned long)
#define SHARE_MEMORY_FREE           _IOWR('m', 3, unsigned long)
#define SHARE_MEMORY_SHOW           _IOWR('m', 4, unsigned long)
#define SHARE_MEMORY_INVAL_RANGE    _IOWR('m', 5, unsigned long)
#define SHARE_MEMORY_WB_RANGE       _IOWR('m', 6, unsigned long)
#define MEMORY_TEST_BLOCK_SIZE      4096        /* 测试申请的内存空间大小 */
#define MEMORY_TEST_BLOCK_ALIGN     4096        /* 如果需要mmap映射,对齐需要4K的整数倍 */
#define SHARE_MEMORY_DEV            "/dev/k510-share-memory"
#define MAP_MEMORY_DEV              "/dev/mem"


#define PADDING_R 114
#define PADDING_G 114
#define PADDING_B 114
struct share_memory_alloc_align_args {
    uint32_t size;
    uint32_t alignment;
    uint32_t phyAddr;
};

typedef struct BoxInfo
{
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    int label;
} BoxInfo;

typedef struct Framesize
{
    int width;
    int height;
} Framesize;

typedef struct ai_worker_args
{
    char* kmodel_path;
    int net_len;
    int valid_width;
    int valid_height;
    float obj_thresh;
    float nms_thresh;
    float iou_distance;    //new,IOU_DISTANCE
    int is_rgb;
    int enable_profile;
    int max_n_init;        //new,MAX_N_INIT
    int max_hits;          //new
    std::string dump_img_dir;
    bool enable_rtsp;
}ai_worker_args;

std::vector<BoxInfo> decode_infer(float* data, int net_size, int stride, int num_classes, Framesize frame_size, float anchors[][2], float threshold);
void nms(std::vector<BoxInfo>& input_boxes, float NMS_THRESH, DETECTION_ROWS& output_boxes);
class ScopedTiming
{
public:
    ScopedTiming(std::string info = "ScopedTiming", int enable_profile = 1)
        : m_info(info), enable_profile(enable_profile)
    {
        if(enable_profile)
        {
            m_start = std::chrono::steady_clock::now();
        }
    }

    ~ScopedTiming()
    {
        if(enable_profile)
        {
            m_stop = std::chrono::steady_clock::now();
            double elapsed_ms = std::chrono::duration<double, std::milli>(m_stop - m_start).count();
            std::cout << m_info << " took " << elapsed_ms << " ms" << std::endl;
        }
    }

private:
    int enable_profile;
    std::string m_info;
    std::chrono::steady_clock::time_point m_start;
    std::chrono::steady_clock::time_point m_stop;
};
#endif