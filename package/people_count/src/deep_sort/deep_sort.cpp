/*
 * VideoTracker.cpp
 *
 *  Created on: Dec 15, 2017
 *      Author: zy
 */
#include <fstream>
#include "deep_sort.h"
#include "opencv2/opencv.hpp"
#include "matching/tracker.h"
#include "definition/config.h"
#include"deep_sort.h"
int rl_count = 0;
DeepSort::DeepSort(ai_worker_args ai_args)
{
    // parse ai worker agrs
    // parse ai worker agrs
    char* kmodel_path = ai_args.kmodel_path;  // object detection kmodel path
    int net_len = ai_args.net_len;  // object detection kmodel input size is net_len * net_len
    int valid_width = ai_args.valid_width;  // isp ds2 input width, should be the same with definition in video config
    int valid_height = ai_args.valid_height;  // isp ds2 input height, should be the same with definition in video config
    float obj_thresh = ai_args.obj_thresh;  // object detection thresh
    float nms_thresh = ai_args.nms_thresh;  // object detection nms thresh
    int is_rgb = ai_args.is_rgb;  // isp ds2 input format, RGB or BGR, RGB now
    int enable_profile = ai_args.enable_profile;  // wether enable time counting
    std::string dump_img_dir = ai_args.dump_img_dir;  // where to dump image 
    this->offset_channel = ai_args.valid_width * ai_args.valid_width;  // ds2 channel offset
    this->enable_dump_image = (dump_img_dir != "None");
    
    this->ai_args = ai_args;
    this->obj_det = new objectDetect(obj_thresh, nms_thresh, net_len, { ai_args.valid_width, ai_args.valid_height });
    this->obj_det->load_model(kmodel_path);
    this->obj_det->prepare_memory();

    this->deep_sort_tracker = new tracker(ai_args.iou_distance, ai_args.max_n_init,ai_args.max_hits); 
}

void DeepSort::run_with_det_model(cv::Mat& img, DEEP_SORT_RET& ret)
{
    if (this->ai_args.is_rgb)
    {
        // R
        memset(this->obj_det->virtual_addr_input[0], PADDING_R, (this->ai_args.valid_width - this->ai_args.valid_height) / 2 * this->ai_args.valid_width);
        memset(this->obj_det->virtual_addr_input[0] + ((this->ai_args.valid_width - this->ai_args.valid_height) / 2 + this->ai_args.valid_height) * this->ai_args.valid_width, PADDING_R, (this->ai_args.valid_width - (this->ai_args.valid_width - this->ai_args.valid_height) / 2 - this->ai_args.valid_height) * this->ai_args.valid_width);
        // G
        memset(this->obj_det->virtual_addr_input[0] + offset_channel, PADDING_G, (this->ai_args.valid_width - this->ai_args.valid_height) / 2 * this->ai_args.valid_width);
        memset(this->obj_det->virtual_addr_input[0] + offset_channel + ((this->ai_args.valid_width - this->ai_args.valid_height) / 2 + this->ai_args.valid_height) * this->ai_args.valid_width, PADDING_G, (this->ai_args.valid_width - (this->ai_args.valid_width - this->ai_args.valid_height) / 2 - this->ai_args.valid_height) * this->ai_args.valid_width);
        // B
        memset(this->obj_det->virtual_addr_input[0] + offset_channel * 2, PADDING_B, (this->ai_args.valid_width - this->ai_args.valid_height) / 2 * this->ai_args.valid_width);
        memset(this->obj_det->virtual_addr_input[0] + offset_channel * 2 + ((this->ai_args.valid_width - this->ai_args.valid_height) / 2 + this->ai_args.valid_height) * this->ai_args.valid_width, PADDING_B, (this->ai_args.valid_width - (this->ai_args.valid_width - this->ai_args.valid_height) / 2 - this->ai_args.valid_height) * this->ai_args.valid_width);
    }
    else
    {
        // B
        memset(this->obj_det->virtual_addr_input[0], PADDING_B, (this->ai_args.valid_width - this->ai_args.valid_height) / 2 * this->ai_args.valid_width);
        memset(this->obj_det->virtual_addr_input[0] + ((this->ai_args.valid_width - this->ai_args.valid_height) / 2 + this->ai_args.valid_height) * this->ai_args.valid_width, PADDING_B, (this->ai_args.valid_width - (this->ai_args.valid_width - this->ai_args.valid_height) / 2 - this->ai_args.valid_height) * this->ai_args.valid_width);
        // G
        memset(this->obj_det->virtual_addr_input[0] + offset_channel, PADDING_G, (this->ai_args.valid_width - this->ai_args.valid_height) / 2 * this->ai_args.valid_width);
        memset(this->obj_det->virtual_addr_input[0] + offset_channel + ((this->ai_args.valid_width - this->ai_args.valid_height) / 2 + this->ai_args.valid_height) * this->ai_args.valid_width, PADDING_G, (this->ai_args.valid_width - (this->ai_args.valid_width - this->ai_args.valid_height) / 2 - this->ai_args.valid_height) * this->ai_args.valid_width);
        // R
        memset(this->obj_det->virtual_addr_input[0] + offset_channel * 2, PADDING_R, (this->ai_args.valid_width - this->ai_args.valid_height) / 2 * this->ai_args.valid_width);
        memset(this->obj_det->virtual_addr_input[0] + offset_channel * 2 + ((this->ai_args.valid_width - this->ai_args.valid_height) / 2 + this->ai_args.valid_height) * this->ai_args.valid_width, PADDING_R, (this->ai_args.valid_width - (this->ai_args.valid_width - this->ai_args.valid_height) / 2 - this->ai_args.valid_height) * this->ai_args.valid_width);

    }
    if (enable_dump_image)
    {
        cv::Mat padding_img_R = cv::Mat(this->ai_args.valid_width, this->ai_args.valid_width, CV_8UC1, this->obj_det->virtual_addr_input[0]);
        cv::Mat padding_img_G = cv::Mat(this->ai_args.valid_width, this->ai_args.valid_width, CV_8UC1, this->obj_det->virtual_addr_input[0] + offset_channel);
        cv::Mat padding_img_B = cv::Mat(this->ai_args.valid_width, this->ai_args.valid_width, CV_8UC1, this->obj_det->virtual_addr_input[0] + offset_channel * 2);
        std::vector<cv::Mat>padding_imgparts(3);
        padding_imgparts.clear();
        padding_imgparts.push_back(padding_img_B);
        padding_imgparts.push_back(padding_img_G);
        padding_imgparts.push_back(padding_img_R);
        cv::Mat padding_img;
        cv::merge(padding_imgparts, padding_img);
        std::string padding_img_out_path = this->ai_args.dump_img_dir + "/padding_img_" + std::to_string(rl_count++) + ".jpg";
        cv::imwrite(padding_img_out_path, padding_img);
    }

    this->obj_det->set_input(0,img);
    this->obj_det->set_output();

    {
        ScopedTiming st("od run", this->ai_args.enable_profile);
        this->obj_det->run();
    }

    {
        ScopedTiming st("od get output", this->ai_args.enable_profile);
        this->obj_det->get_output();
    }

    std::vector<BoxInfo> result;
    DETECTION_ROWS detections;
    {
        ScopedTiming st("post process", this->ai_args.enable_profile);
        this->obj_det->post_process(result, detections);
    }

    // std::cout<<"result start"<<std::endl;
    // for(auto iter=result.begin();iter!=result.end();iter++)
    // {
    //      std::cout<<"result:"<<iter->label<<" "<<iter->score<<" "<<iter->x1<<" "<<iter->y1<<" "<<iter->x2<<" "<<iter->y2<<std::endl;
    // }
   
    this->deep_sort_tracker->predict();
    this->deep_sort_tracker->update(detections, ret);


}

void DeepSort::run_with_det_model_isp(DEEP_SORT_RET& ret)
{    
    if (this->ai_args.is_rgb)
    {
        // R
        memset(this->obj_det->virtual_addr_input[0], PADDING_R, (this->ai_args.valid_width - this->ai_args.valid_height) / 2 * this->ai_args.valid_width);
        memset(this->obj_det->virtual_addr_input[0] + ((this->ai_args.valid_width - this->ai_args.valid_height) / 2 + this->ai_args.valid_height) * this->ai_args.valid_width, PADDING_R, (this->ai_args.valid_width - (this->ai_args.valid_width - this->ai_args.valid_height) / 2 - this->ai_args.valid_height) * this->ai_args.valid_width);
        // G
        memset(this->obj_det->virtual_addr_input[0] + offset_channel, PADDING_G, (this->ai_args.valid_width - this->ai_args.valid_height) / 2 * this->ai_args.valid_width);
        memset(this->obj_det->virtual_addr_input[0] + offset_channel + ((this->ai_args.valid_width - this->ai_args.valid_height) / 2 + this->ai_args.valid_height) * this->ai_args.valid_width, PADDING_G, (this->ai_args.valid_width - (this->ai_args.valid_width - this->ai_args.valid_height) / 2 - this->ai_args.valid_height) * this->ai_args.valid_width);
        // B
        memset(this->obj_det->virtual_addr_input[0] + offset_channel * 2, PADDING_B, (this->ai_args.valid_width - this->ai_args.valid_height) / 2 * this->ai_args.valid_width);
        memset(this->obj_det->virtual_addr_input[0] + offset_channel * 2 + ((this->ai_args.valid_width - this->ai_args.valid_height) / 2 + this->ai_args.valid_height) * this->ai_args.valid_width, PADDING_B, (this->ai_args.valid_width - (this->ai_args.valid_width - this->ai_args.valid_height) / 2 - this->ai_args.valid_height) * this->ai_args.valid_width);
    }
    else
    {
        // B
        memset(this->obj_det->virtual_addr_input[0], PADDING_B, (this->ai_args.valid_width - this->ai_args.valid_height) / 2 * this->ai_args.valid_width);
        memset(this->obj_det->virtual_addr_input[0] + ((this->ai_args.valid_width - this->ai_args.valid_height) / 2 + this->ai_args.valid_height) * this->ai_args.valid_width, PADDING_B, (this->ai_args.valid_width - (this->ai_args.valid_width - this->ai_args.valid_height) / 2 - this->ai_args.valid_height) * this->ai_args.valid_width);
        // G
        memset(this->obj_det->virtual_addr_input[0] + offset_channel, PADDING_G, (this->ai_args.valid_width - this->ai_args.valid_height) / 2 * this->ai_args.valid_width);
        memset(this->obj_det->virtual_addr_input[0] + offset_channel + ((this->ai_args.valid_width - this->ai_args.valid_height) / 2 + this->ai_args.valid_height) * this->ai_args.valid_width, PADDING_G, (this->ai_args.valid_width - (this->ai_args.valid_width - this->ai_args.valid_height) / 2 - this->ai_args.valid_height) * this->ai_args.valid_width);
        // R
        memset(this->obj_det->virtual_addr_input[0] + offset_channel * 2, PADDING_R, (this->ai_args.valid_width - this->ai_args.valid_height) / 2 * this->ai_args.valid_width);
        memset(this->obj_det->virtual_addr_input[0] + offset_channel * 2 + ((this->ai_args.valid_width - this->ai_args.valid_height) / 2 + this->ai_args.valid_height) * this->ai_args.valid_width, PADDING_R, (this->ai_args.valid_width - (this->ai_args.valid_width - this->ai_args.valid_height) / 2 - this->ai_args.valid_height) * this->ai_args.valid_width);

    }
    if (enable_dump_image)
    {
        cv::Mat padding_img_R = cv::Mat(this->ai_args.valid_width, this->ai_args.valid_width, CV_8UC1, this->obj_det->virtual_addr_input[0]);
        cv::Mat padding_img_G = cv::Mat(this->ai_args.valid_width, this->ai_args.valid_width, CV_8UC1, this->obj_det->virtual_addr_input[0] + offset_channel);
        cv::Mat padding_img_B = cv::Mat(this->ai_args.valid_width, this->ai_args.valid_width, CV_8UC1, this->obj_det->virtual_addr_input[0] + offset_channel * 2);
        std::vector<cv::Mat>padding_imgparts(3);
        padding_imgparts.clear();
        padding_imgparts.push_back(padding_img_B);
        padding_imgparts.push_back(padding_img_G);
        padding_imgparts.push_back(padding_img_R);
        cv::Mat padding_img;
        cv::merge(padding_imgparts, padding_img);
        
        std::string padding_img_out_path = this->ai_args.dump_img_dir + "/padding_img_" + std::to_string(rl_count++) + ".jpg";
        cv::imwrite(padding_img_out_path, padding_img);
    }

    this->obj_det->set_input(0);
    this->obj_det->set_output();

    {
        ScopedTiming st("od run", this->ai_args.enable_profile);
        this->obj_det->run();
    }

    {
        ScopedTiming st("od get output", this->ai_args.enable_profile);
        this->obj_det->get_output();
    }
    
    std::vector<BoxInfo> result;
    DETECTION_ROWS detections;
    {
        ScopedTiming st("post process", this->ai_args.enable_profile);
        this->obj_det->post_process(result, detections);
    }
    this->deep_sort_tracker->predict();
    this->deep_sort_tracker->update(detections, ret);
}

objectDetect* DeepSort::get_det()
{
    return this->obj_det;
}

tracker* DeepSort::get_tracker()
{
    return this->deep_sort_tracker;
}

DeepSort::~DeepSort()
{
    delete this->obj_det;
    delete this->deep_sort_tracker;
}