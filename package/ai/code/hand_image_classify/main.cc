/* Copyright (c) 2022, Canaan Bright Sight Co., Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <nncase/runtime/host_runtime_tensor.h>
#include <nncase/runtime/interpreter.h>
#include <nncase/runtime/runtime_tensor.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utils/logger.hpp>

#include "string.h"
#include <signal.h>
/*  进程优先级  */
#include <unistd.h>
#include <sched.h>
#include <pthread.h>
#include <thread>
#include <mutex>
/* 申请物理内存 */
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <atomic>
#include<vector>

#include "k510_drm.h"
#include "media_ctl.h"
#include <linux/videodev2.h>


#include "yolo.h"
#include "hld.h"
#include "imagenet.h"
#include "cv2_utils.h"
extern int hls_hand[12][2];
extern int connections[15][2];


struct video_info dev_info[2];
std::mutex mtx;
uint8_t drm_bufs_index = 0;
uint8_t drm_bufs_argb_index = 0;
struct drm_buffer *fbuf_yuv, *fbuf_argb;
int obj_cnt;

std::atomic<bool> quit(true);

void fun_sig(int sig)
{
    if(sig == SIGINT)
    {
        quit.store(false);
    }
}

void ai_worker(ai_worker_args ai_args)
{
    // parse ai worker agrs
    char* hd_kmodel_path = ai_args.hd_kmodel_path;  // hand_detection kmodel path
    int hd_net_len = ai_args.hd_net_len;  // hand_detection kmodel input size is net_len * net_len
    int valid_width = ai_args.valid_width;  // isp ds2 input width, should be the same with definition in video config
    int valid_height = ai_args.valid_height;  // isp ds2 input height, should be the same with definition in video config
    float obj_thresh = ai_args.obj_thresh;  // hand_detection thresh
    float nms_thresh = ai_args.nms_thresh;  // hand_detection nms thresh
    char* hld_kmodel_path = ai_args.hld_kmodel_path;  // hand lamdmark kmodel path
    int hld_net_len = ai_args.hld_net_len;  //hand landmark kmodel input size
    int hld_num = ai_args.hld_num;  // hand landmark number
    char* iclass_kmodel_path = ai_args.iclass_kmodel_path;  // classification kmodel path
    int iclass_net_len = ai_args.iclass_net_len;  // classification kmodel input size
    int iclass_num = ai_args.iclass_num;  // classification numbers
    char* iclass_labels_path = ai_args.iclass_labels_path;  // label path
    int is_rgb = ai_args.is_rgb;  // isp ds2 input format, RGB or BGR, RGB now
    int enable_profile = ai_args.enable_profile;  // wether enable time counting
    std::string dump_img_dir = ai_args.dump_img_dir;  // where to dump image 
    int offset_channel = valid_width * valid_width;  // ds2 channel offset
    int enable_dump_image = (dump_img_dir != "None");
    yolo hd(hd_net_len, obj_thresh, nms_thresh);
    hd.load_model(hd_kmodel_path);  // load kmodel
	hd.prepare_memory();  // memory allocation
    hld hkd(hld_net_len, hld_num);
    hkd.load_model(hld_kmodel_path);  // load kmodel
	hkd.prepare_memory();  // memory allocation
    imagenet iclass(iclass_net_len, iclass_num);    
    iclass.load_model(iclass_kmodel_path);  // load kmodel
	iclass.prepare_memory();  // memory allocation
    iclass.get_label(iclass_labels_path);
    
    /****fixed operation for video operation****/
    mtx.lock();
    cv::VideoCapture capture;
	capture.open(5);
    // video setting
    capture.set(cv::CAP_PROP_CONVERT_RGB, 0);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, hd_net_len);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, hd_net_len);
    // RRRRRR....GGGGGGG....BBBBBB, CHW
    capture.set(cv::CAP_PROP_FOURCC, V4L2_PIX_FMT_RGB24);
    mtx.unlock();


    int frame_cnt = 0;

    // define cv::Mat for ai input
    // padding offset is (valid_width - valid_height) / 2 * valid_width
    cv::Mat rgb24_img_for_ai(hd_net_len, hd_net_len, CV_8UC3, hd.virtual_addr_input[0] + (valid_width - valid_height) / 2 * valid_width);
    // define cv::Mat for post process
    cv::Mat ori_img_R = cv::Mat(valid_height, valid_width, CV_8UC1, hd.virtual_addr_input[0] + (valid_width - valid_height) / 2 * valid_width);
    cv::Mat ori_img_G = cv::Mat(valid_height, valid_width, CV_8UC1, hd.virtual_addr_input[0] + (valid_width - valid_height) / 2 * valid_width + valid_width * valid_width);
    cv::Mat ori_img_B = cv::Mat(valid_height, valid_width, CV_8UC1, hd.virtual_addr_input[0] + (valid_width - valid_height) / 2 * valid_width + valid_width * valid_width * 2);
    

    cv::Mat hld_img_R = cv::Mat(hld_net_len, hld_net_len, CV_8UC1, hkd.virtual_addr_input[0]);
    cv::Mat hld_img_G = cv::Mat(hld_net_len, hld_net_len, CV_8UC1, hkd.virtual_addr_input[0] + hld_net_len * hld_net_len);
    cv::Mat hld_img_B = cv::Mat(hld_net_len, hld_net_len, CV_8UC1, hkd.virtual_addr_input[0] + hld_net_len * hld_net_len * 2);

    cv::Mat iclass_img_R = cv::Mat(iclass_net_len, iclass_net_len, CV_8UC1, iclass.virtual_addr_input[0]);
    cv::Mat iclass_img_G = cv::Mat(iclass_net_len, iclass_net_len, CV_8UC1, iclass.virtual_addr_input[0] + iclass_net_len * iclass_net_len);
    cv::Mat iclass_img_B = cv::Mat(iclass_net_len, iclass_net_len, CV_8UC1, iclass.virtual_addr_input[0] + iclass_net_len * iclass_net_len * 2);

    cv::Mat cropped_R;
    cv::Mat cropped_G;
    cv::Mat cropped_B;
    box_t cropped_box;
    std::vector<Point> left_8;
    std::vector<Point> right_8;
    float* output_joints_dst = new float[hld_num * 2];
    frame_coordinate_info frame_coordinate;
    while(quit.load()) 
    {
        ScopedTiming st("total", 1);
        mtx.lock();
        capture.read(rgb24_img_for_ai);
        mtx.unlock();
        //拷贝图像的同时修改padding方式，默认读出的图像是最后做padding，修改为前后做padding
        if(is_rgb)
        {
            // R
            memset(hd.virtual_addr_input[0], PADDING_R, (valid_width - valid_height) / 2 * valid_width);
            memset(hd.virtual_addr_input[0] + ((valid_width - valid_height) / 2 + valid_height) * valid_width, PADDING_R, (valid_width - (valid_width - valid_height) / 2 - valid_height) * valid_width);
            // G
            memset(hd.virtual_addr_input[0] + offset_channel, PADDING_G, (valid_width - valid_height) / 2 * valid_width);
            memset(hd.virtual_addr_input[0] + offset_channel + ((valid_width - valid_height) / 2 + valid_height) * valid_width, PADDING_G, (valid_width - (valid_width - valid_height) / 2 - valid_height) * valid_width);
            // B
            memset(hd.virtual_addr_input[0] + offset_channel * 2, PADDING_B, (valid_width - valid_height) / 2 * valid_width);
            memset(hd.virtual_addr_input[0] + offset_channel * 2 + ((valid_width - valid_height) / 2 + valid_height) * valid_width, PADDING_B, (valid_width - (valid_width - valid_height) / 2 - valid_height) * valid_width);    
        }
        else
        {
            // B
            memset(hd.virtual_addr_input[0], PADDING_B, (valid_width - valid_height) / 2 * valid_width);
            memset(hd.virtual_addr_input[0] + ((valid_width - valid_height) / 2 + valid_height) * valid_width, PADDING_B, (valid_width - (valid_width - valid_height) / 2 - valid_height) * valid_width);
            // G
            memset(hd.virtual_addr_input[0] + offset_channel, PADDING_G, (valid_width - valid_height) / 2 * valid_width);
            memset(hd.virtual_addr_input[0] + offset_channel + ((valid_width - valid_height) / 2 + valid_height) * valid_width, PADDING_G, (valid_width - (valid_width - valid_height) / 2 - valid_height) * valid_width);
            // R
            memset(hd.virtual_addr_input[0] + offset_channel * 2, PADDING_R, (valid_width - valid_height) / 2 * valid_width);
            memset(hd.virtual_addr_input[0] + offset_channel * 2 + ((valid_width - valid_height) / 2 + valid_height) * valid_width, PADDING_R, (valid_width - (valid_width - valid_height) / 2 - valid_height) * valid_width);
            
        }
        if(enable_dump_image)
        {
            std::vector<cv::Mat>ori_imgparts(3);
            ori_imgparts.clear();
            ori_imgparts.push_back(ori_img_B);
            ori_imgparts.push_back(ori_img_G);
            ori_imgparts.push_back(ori_img_R);
            cv::Mat ori_img;
            cv::merge(ori_imgparts, ori_img);
            std::string ori_img_out_path = dump_img_dir + "/ori_img_" + std::to_string(frame_cnt) + ".jpg";
            cv::imwrite(ori_img_out_path, ori_img);
        }
        
        hd.set_input(0);
        hd.set_output();
        hkd.set_input(0);
        hkd.set_output();
        iclass.set_input(0);
        iclass.set_output();

        {
            ScopedTiming st("hd run", enable_profile);
            hd.run();
        }

        {
            ScopedTiming st("hd get output", enable_profile);
            hd.get_output();
            hd.post_process();
        }

        std::vector<box_t> valid_box;
        {
            ScopedTiming st("get final box", enable_profile);
            hd.get_final_box(valid_box);
        }

        /****fixed operation for display clear****/
        cv::Mat img_argb;
        {
            ScopedTiming st("display clear", enable_profile);
            fbuf_argb = &drm_dev.drm_bufs_argb[drm_bufs_argb_index];
            img_argb = cv::Mat(DRM_INPUT_HEIGHT, DRM_INPUT_WIDTH, CV_8UC4, (uint8_t *)fbuf_argb->map);

            for(uint32_t i = 0; i < obj_cnt; i++)
            {
                if(i == 0)
                {
                    img_argb.setTo(cv::Scalar(0, 0, 0, 0));
                }
                struct vo_draw_frame frame;
                frame.crtc_id = drm_dev.crtc_id;
                frame.draw_en = 0;
                frame.frame_num = i;
                draw_frame(&frame);
            }
        }

        {
            ScopedTiming st("loop found box", enable_profile);
            obj_cnt = 0;
            for (auto b : valid_box)
            {
                cropped_box = scale_coords(hd_net_len, b, ori_img_R);
                {
                    ScopedTiming st("hld process", enable_profile);
                    hkd.set_valid_box(cropped_box, ori_img_R);
                }
                cropped_R = hkd.crop_image(ori_img_R);
                cv::resize(cropped_R, hld_img_R, cv::Size(hld_net_len, hld_net_len), cv::INTER_AREA);
                cropped_G = hkd.crop_image(ori_img_G);
                cv::resize(cropped_G, hld_img_G, cv::Size(hld_net_len, hld_net_len), cv::INTER_AREA);
                cropped_B = hkd.crop_image(ori_img_B);
                cv::resize(cropped_B, hld_img_B, cv::Size(hld_net_len, hld_net_len), cv::INTER_AREA);              
                {
                    ScopedTiming st("hld run", enable_profile);
                    hkd.run();
                }
                {
                    ScopedTiming st("hld post process", enable_profile);                    
                    float *pts = reinterpret_cast<float *>(hkd.virtual_addr_output);
                    frame_coordinate = get_frame_coord(cropped_box, valid_width, valid_height);
                    float xy = sqrtf(powf(1.0 * (frame_coordinate.endx - frame_coordinate.startx), 2) + powf(1.0 * (frame_coordinate.endy - frame_coordinate.starty), 2));
                    for (uint32_t pp = 0; pp < hld_num; pp++)
                    {
                        output_joints_dst[2 * pp + 0] = pts[2 * pp + 0] * hkd.valid_box.w + hkd.valid_box.x;
                        output_joints_dst[2 * pp + 1] = pts[2 * pp + 1] * hkd.valid_box.h + hkd.valid_box.y;
                        output_joints_dst[2 * pp + 0] = output_joints_dst[2 * pp + 0] / valid_width * img_argb.cols;
                        output_joints_dst[2 * pp + 1] = output_joints_dst[2 * pp + 1] / valid_height * img_argb.rows;
                    }
                    Point temp = {output_joints_dst[2 * 8 + 0], output_joints_dst[2 * 8 + 1]};
                    if(valid_box.size() == 2)
                    {                        
                        if(left_8.size() == 0 || right_8.size() == 0)
                        {
                            if(obj_cnt == 0)
                            {
                                left_8.push_back(temp);
                            }
                            else
                            {
                                right_8.push_back(temp);
                            }
                        }
                        else
                        {
                            Point left_last = left_8[left_8.size() - 1];
                            float delta_left = sqrtf(powf(temp.x - left_last.x, 2) + powf(temp.y - left_last.y, 2));
                            Point right_last = right_8[right_8.size() - 1];
                            float delta_right = sqrtf(powf(temp.x - right_last.x, 2) + powf(temp.y - right_last.y, 2));
                            int delta_min_index = 0;
                            float delta_min = delta_left;
                            if(delta_left > delta_right)
                            {
                                delta_min_index = 1;
                                delta_min = delta_right;
                            }
                            if(delta_min < 0.15 * xy)
                            {
                                if(delta_min_index == 0 && left_8.size() < 10)
                                {
                                    left_8.push_back(temp);
                                }
                                if(delta_min_index == 1 && right_8.size() < 10)
                                {
                                    right_8.push_back(temp);
                                }
                            }
                            else
                            {
                                left_8.clear();
                                right_8.clear();
                            }
                        }
                    }   
                }
                {
                    ScopedTiming st("draw osd", enable_profile);  
                    if (obj_cnt < 32)
                    {                             
                        for (uint32_t kk = 0; kk < hld_num; kk++)
                        {
                            cv::Point point_draw;
                            point_draw.x = int(output_joints_dst[kk * 2 + 0]);
                            point_draw.y = int(output_joints_dst[kk * 2 + 1]);
                            cv::circle(img_argb, point_draw, 8, cv::Scalar(255, 0, 0, 255), -1);    
                        }
                        for (uint32_t kk = 0; kk < 15; kk++)
                        {
                            cv::Point point0, point1;
                            int kindex0 = connections[kk][0];
                            int kindex1 = connections[kk][1];
                            point0.x = int(output_joints_dst[kindex0 * 2 + 0]);
                            point0.y = int(output_joints_dst[kindex0 * 2 + 1]);
                            point1.x = int(output_joints_dst[kindex1 * 2 + 0]);
                            point1.y = int(output_joints_dst[kindex1 * 2 + 1]);
                            cv::line(img_argb, point0, point1, cv::Scalar(255, 0, 255, 255), 4);
                        }
                        for (uint32_t kk = 0; kk < 12; kk++)
                        {
                            cv::Point point0, point1;
                            int kindex0 = hls_hand[kk][0];
                            int kindex1 = hls_hand[kk][1];
                            point0.x = int(output_joints_dst[kindex0 * 2 + 0]);
                            point0.y = int(output_joints_dst[kindex0 * 2 + 1]);
                            point1.x = int(output_joints_dst[kindex1 * 2 + 0]);
                            point1.y = int(output_joints_dst[kindex1 * 2 + 1]);
                            cv::line(img_argb, point0, point1, cv::Scalar(0, 255, 255, 255), 4);
                        }
                    }
                }                
                if(enable_dump_image)
                {   
                    std::vector<cv::Mat>cropped_imgparts(3);                  
                    cropped_imgparts.clear();
                    cropped_imgparts.push_back(cropped_B);
                    cropped_imgparts.push_back(cropped_G);
                    cropped_imgparts.push_back(cropped_R); 
                    cv::Mat cropped_img;
                    cv::merge(cropped_imgparts, cropped_img);
                    std::string cropped_image_out_path = dump_img_dir + "/hcropped_" + std::to_string(frame_cnt) + "_" + std::to_string(obj_cnt) + ".jpg";
                    cv::imwrite(cropped_image_out_path, cropped_img); 
                    std::vector<cv::Mat>resized_imgparts(3);
                    resized_imgparts.clear();
                    resized_imgparts.push_back(hld_img_B);
                    resized_imgparts.push_back(hld_img_G);          
                    resized_imgparts.push_back(hld_img_R);
                    cv::Mat resized_img;
                    cv::merge(resized_imgparts, resized_img);
                    std::string resized_image_out_path = dump_img_dir + "/hresized_" + std::to_string(frame_cnt) + "_" + std::to_string(obj_cnt) + ".jpg";
                    cv::imwrite(resized_image_out_path, resized_img); 
                }
                obj_cnt += 1;
            }
            if(left_8.size() != 0)
            {
                int fill_cnt = int(left_8.size());
                cv::Point point_draw;
                point_draw.x = static_cast<int>(left_8[0].x);
                point_draw.y = static_cast<int>(left_8[0].y);
                if (fill_cnt != 10)
                {
                    cv::ellipse(img_argb, point_draw, cv::Size(64, 64), 0, 0, fill_cnt * 36, (255, 255, 0, 255), 8);
                }
                else
                {
                    cv::ellipse(img_argb, point_draw, cv::Size(64, 64), 0, 0, fill_cnt * 36, (255, 255, 255, 255), 16);
                }
            }
            if(right_8.size() != 0)
            {
                int fill_cnt = int(right_8.size());
                cv::Point point_draw;
                point_draw.x = static_cast<int>(right_8[0].x);
                point_draw.y = static_cast<int>(right_8[0].y);
                if (fill_cnt != 10)
                {
                    cv::ellipse(img_argb, point_draw, cv::Size(64, 64), 0, 0, fill_cnt * 36, (255, 255, 0, 255), 8, -1);
                }
                else
                {
                    cv::ellipse(img_argb, point_draw, cv::Size(64, 64), 0, 0, fill_cnt * 36, (255, 255, 255, 255), 16, -1);
                }
            }
            if(left_8.size() == 10 && right_8.size() == 10)
            {
                if(valid_box.size() != 2)
                {
                    left_8.clear();
                    right_8.clear();                            
                }
                else
                {
                    int choose_xmin = static_cast<int>(left_8[0].x);
                    int choose_xmax = static_cast<int>(right_8[0].x);
                    int choose_ymin = static_cast<int>(left_8[0].y);
                    int choose_ymax = static_cast<int>(right_8[0].y);
                    int startx = std::min(img_argb.cols, std::min(choose_xmin, choose_xmax)); 
                    int endx = std::min(img_argb.cols, std::max(choose_xmin, choose_xmax)); 
                    int starty = std::min(img_argb.rows, std::min(choose_ymin, choose_ymax)); 
                    int endy = std::min(img_argb.rows, std::max(choose_ymin, choose_ymax)); 
                    cv::circle(img_argb, cv::Point(choose_xmin, choose_ymin), 16, cv::Scalar(255, 255, 255, 255), -1);  
                    cv::circle(img_argb, cv::Point(choose_xmax, choose_ymax), 16, cv::Scalar(255, 255, 255, 255), -1);  
                    cv::rectangle(img_argb, cv::Point(startx,starty), cv::Point(endx,endy), cv::Scalar(255, 0, 0, 255), 4, 1, 0);
                    cropped_box.x = 1.0 * startx / img_argb.cols * valid_width;
                    cropped_box.w = 1.0 * endx / img_argb.cols * valid_width;
                    cropped_box.y = 1.0 * starty / img_argb.rows * valid_height;
                    cropped_box.h = 1.0 * endy / img_argb.rows * valid_height;
                    {
                        ScopedTiming st("iclass process", enable_profile);
                        iclass.set_valid_box(cropped_box, ori_img_R);
                    }
                    cropped_R = iclass.crop_image(ori_img_R);
                    cv::resize(cropped_R, iclass_img_R, cv::Size(iclass_net_len, iclass_net_len), cv::INTER_AREA);
                    cropped_G = iclass.crop_image(ori_img_G);
                    cv::resize(cropped_G, iclass_img_G, cv::Size(iclass_net_len, iclass_net_len), cv::INTER_AREA);
                    cropped_B = iclass.crop_image(ori_img_B);
                    cv::resize(cropped_B, iclass_img_B, cv::Size(iclass_net_len, iclass_net_len), cv::INTER_AREA);              
                    {
                        ScopedTiming st("iclass run", enable_profile);
                        iclass.run();
                    }
                    {
                        ScopedTiming st("iclass post process", enable_profile);
                        iclass.post_process();
                    }
                    std::string text = iclass.labels[iclass.idx] + ":" + std::to_string(iclass.prob).substr(0,4);
                    cv::putText(img_argb, text, cv::Point(startx, starty), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(255, 0, 0, 255), 2, 8, 0);
                }                
                if(enable_dump_image)
                {                        
                    std::vector<cv::Mat>cropped_imgparts(3);                  
                    cropped_imgparts.clear();
                    cropped_imgparts.push_back(cropped_B);
                    cropped_imgparts.push_back(cropped_G);
                    cropped_imgparts.push_back(cropped_R); 
                    cv::Mat cropped_img;
                    cv::merge(cropped_imgparts, cropped_img);
                    std::string cropped_image_out_path = dump_img_dir + "/icropped_" + std::to_string(frame_cnt) + ".jpg";
                    cv::imwrite(cropped_image_out_path, cropped_img); 
                    std::vector<cv::Mat>resized_imgparts(3);
                    resized_imgparts.clear();
                    resized_imgparts.push_back(hld_img_B);
                    resized_imgparts.push_back(hld_img_G);          
                    resized_imgparts.push_back(hld_img_R);
                    cv::Mat resized_img;
                    cv::merge(resized_imgparts, resized_img);
                    std::string resized_image_out_path = dump_img_dir + "/iresized_" + std::to_string(frame_cnt) + ".jpg";
                    cv::imwrite(resized_image_out_path, resized_img); 
                }
            }
        }
        frame_cnt += 1;
        drm_bufs_argb_index = !drm_bufs_argb_index;
    }    
    delete[] output_joints_dst;
    /****fixed operation for capture release and display clear****/
    printf("%s ==========release \n", __func__);
    mtx.lock();
    capture.release();
    mtx.unlock();
    for(uint32_t i = 0; i < obj_cnt; i++)
    {
        struct vo_draw_frame frame;
        frame.crtc_id = drm_dev.crtc_id;
        frame.draw_en = 0;
        frame.frame_num = i;
        draw_frame(&frame);
    }
}

/****fixed operation for display worker****/
void display_worker(int enable_profile)
{
    mtx.lock();
    cv::VideoCapture capture;
	capture.open(3);
    capture.set(cv::CAP_PROP_CONVERT_RGB, 0);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, (DRM_INPUT_WIDTH + 15) / 16 * 16);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, DRM_INPUT_HEIGHT);
    capture.set(cv::CAP_PROP_FOURCC, V4L2_PIX_FMT_NV12);
    mtx.unlock();
    while(quit.load()) 
    {
        drm_bufs_index = !drm_bufs_index;
        fbuf_yuv = &drm_dev.drm_bufs[drm_bufs_index];
        cv::Mat org_img(DRM_INPUT_HEIGHT * 3 / 2, (DRM_INPUT_WIDTH + 15) / 16 * 16, CV_8UC1, fbuf_yuv->map);
        {
            ScopedTiming st("capture read",enable_profile);
            mtx.lock();
            capture.read(org_img);
            mtx.unlock();
        }

        if (drm_dev.req)
            drm_wait_vsync();
        fbuf_argb = &drm_dev.drm_bufs_argb[!drm_bufs_argb_index];
        if (drm_dmabuf_set_plane(fbuf_yuv, fbuf_argb)) 
        {
            std::cerr << "Flush fail \n";
            goto exit;
        }
    }
exit:
    printf("%s ==========release \n", __func__);
    mtx.lock();
    capture.release();
    mtx.unlock();
}

int main(int argc, char *argv[])
{
    std::cout << "case " << argv[0] << " build " << __DATE__ << " " << __TIME__ << std::endl;
    if (argc < 18)
    {
        std::cerr << "Usage: " << argv[0] << " <hd_kmodel> <hd_net_len> <valid_width> <valid_height> <obj_thresh> <nms_thresh> <hld_kmodel> <hld_net_len> <hld_num> <iclass_kmodel> <iclass_net_len> <iclass_num> <iclass_labels> <video_config> <is_rgb> <enable_profile> <dump_image_dir>" << std::endl;
        return -1;
    }
    // parse args for ai worker
    ai_worker_args ai_args;
    ai_args.hd_kmodel_path = argv[1];
    ai_args.hd_net_len = atoi(argv[2]);
    ai_args.valid_width = atoi(argv[3]);
    ai_args.valid_height = atoi(argv[4]);
    if(ai_args.valid_height > ai_args.valid_width)
    {
        std::cerr << "You should set width bigger than height" << std::endl;
                std::abort();
    }
    if(ai_args.valid_width != ai_args.hd_net_len)
    {
        std::cerr << "We won't resize image for gnne input, so valid_width should be equal to net_len" << std::endl;
                std::abort();
    }
    ai_args.obj_thresh = atof(argv[5]);
    ai_args.nms_thresh = atof(argv[6]);
    ai_args.hld_kmodel_path = argv[7];
    ai_args.hld_net_len = atoi(argv[8]);
    ai_args.hld_num = atoi(argv[9]);
    ai_args.iclass_kmodel_path = argv[10];
    ai_args.iclass_net_len = atoi(argv[11]);
    ai_args.iclass_num = atoi(argv[12]);
    ai_args.iclass_labels_path = argv[13];
    char* video_cfg_file = argv[14];
    ai_args.is_rgb = atoi(argv[15]);
    ai_args.enable_profile = atoi(argv[16]);
    int enable_profile = atoi(argv[16]);
    ai_args.dump_img_dir = argv[17];

    /****fixed operation for ctrl+c****/
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = fun_sig;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);


    /****fixed operation for drm init****/
    drm_init();


    /****fixed operation for mediactl init****/
    mediactl_init(video_cfg_file, &dev_info[0]);


    // create thread for display
    std::thread thread_ds0(display_worker, enable_profile);
    // create thread for ai worker
    std::thread thread_ds2(ai_worker, ai_args);

    thread_ds0.join();
    thread_ds2.join();

    /****fixed operation for drm deinit****/
    for(uint32_t i = 0; i < DRM_BUFFERS_COUNT; i++) 
    {
        drm_destory_dumb(&drm_dev.drm_bufs[i]);
    }
    for(uint32_t i = 0; i < DRM_BUFFERS_COUNT; i++) 
    {
        drm_destory_dumb(&drm_dev.drm_bufs_argb[i]);
    }
    
    return 0;
}