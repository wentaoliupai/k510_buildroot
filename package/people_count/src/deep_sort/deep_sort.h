/* Copyright 2022 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef DEEP_SORT_H
#define DEEP_SORT_H
#include <string>
#include "object_detect.h"
#include "cv2_utils.h"
#include "matching/tracker.h"
 //#include "deep_sort.h"
#include "matching/tracker.h"
#include "definition/config.h"
using std::string;

class DeepSort
{
public:
    DeepSort(ai_worker_args ai_args);

    /***************************************************************
    *  @brief     使用目标检测模型，输入是rgb图像来进行目标跟踪
    *  @param     参数
    *             -img[in] cv::Mat,输入图像，rgb格式
    *             -ret[out] DEEP_SORT_RET,包括消失id列表，所有轨迹的id及xywh坐标
    *             
    *  @note      无
    *  @Sample usage:ai_rgb_worker in main.cc
    **************************************************************/
    void run_with_det_model(cv::Mat& img, DEEP_SORT_RET& ret);
    /***************************************************************
    *  @brief     使用目标检测模型，输入是isp图像来进行目标跟踪
    *  @param     参数
    *             -ret[out] DEEP_SORT_RET,包括消失id列表，所有轨迹的id及xywh坐标
    *
    *  @note      无
    *  @Sample usage:ai_worker in main.cc
    **************************************************************/
    void run_with_det_model_isp(DEEP_SORT_RET& ret);
    ~DeepSort();
    

    /*************************************************
    Function: run_with_det_model_and_reid_model
    Description: 
    Calls: // 被本函数调用的函数清单
    Called By: // 调用本函数的函数清单
    Table Accessed: // 被访问的表（此项仅对于牵扯到数据库操作的程序）
    Table Updated: // 被修改的表（此项仅对于牵扯到数据库操作的程序）
    Input: // 输入参数说明，包括每个参数的作// 用、取值说明及参数间关系。
    Output: // 对输出参数的说明。
    Return: // 函数返回值的说明
    Others: // 其它说明
    *************************************************/

    tracker* get_tracker();
    objectDetect* get_det();
private:
    tracker* deep_sort_tracker;
    objectDetect* obj_det;
    ai_worker_args ai_args;
    int offset_channel;
    int enable_dump_image;
};

#endif 
