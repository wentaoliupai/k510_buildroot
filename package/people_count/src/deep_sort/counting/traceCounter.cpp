#pragma once


#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utils/logger.hpp>
#include "traceCounter.h"
using namespace cv;
#define DRM_BUFFERS_COUNT 3

float g_backGroundBuffer[HEAD_INPUT_IMG_W_H * HEAD_INPUT_IMG_W_H * 3] = { 0 };
extern stru_event_infor event_infor;

//cv::Mat raw_img;
int flag_condition2 = 0;


extern std::vector<std::string> count_to_clear[DRM_BUFFERS_COUNT];
extern std::vector<cv::Point> count_point_to_clear[DRM_BUFFERS_COUNT];
extern std::vector<traceInformation_io> track_to_clear[DRM_BUFFERS_COUNT];

// extern std::vector<cv::Point> points_to_clear;
// extern std::vector<std::string> strs_to_clear;
// extern std::vector<std::string> count_to_clear;
// extern std::vector<cv::Point> count_point_to_clear;
// extern std::vector<traceInformation_io> track_to_clear;

traceInformation_io  trace_infor[TraceNum] = { 0 };


// int lines[4] = { 0,720/2,1080,720/2 };  //横线

int lines[4] = { 1080/2,0,1080/2,1920 };  //竖线

void draw_trace(cv::Mat img,int index)
{
	int i = 0, j = 0;
	static Scalar classic_color[20] = { Scalar(0, 0, 255,255), Scalar(0, 255, 0,255), Scalar(255, 0, 0,255), Scalar(179, 52, 225,255), Scalar(205, 38, 125,255), Scalar(139, 139, 0,255), Scalar(0, 165, 255,255),Scalar(0, 255, 255,255), Scalar(128, 0, 0,255), Scalar(255, 255, 255,255),Scalar(173 ,222 ,255,255),Scalar(139 ,139 ,0,255),Scalar(225 ,228, 255,255),Scalar(205 ,90 ,106,255),Scalar(0,215,255,255),Scalar(34,34,178,255),Scalar(225,225,255,255),Scalar(181,181,181,255),Scalar(144,238,144.255),Scalar(71,99,255,255) };
	for (i = 0; i < TraceNum; i++)
	{
		if (trace_infor[i].count > 0)
		{			
			for (j = 0; j < trace_infor[i].count - 1; j++)
			{
				
				line(img, Point(trace_infor[i].positions[j].x, trace_infor[i].positions[j].y), Point(trace_infor[i].positions[j + 1].x, trace_infor[i].positions[j + 1].y), classic_color[i%TraceNum],2,CV_8S);
				//circle(img, Point(trace_infor[i].positions[j + 1].x, trace_infor[i].positions[j + 1].y), 3, classic_color[i%TraceNum], -1);
				
			}
			// track_to_clear.push_back(trace_infor[i]);
			track_to_clear[index].push_back(trace_infor[i]);
		}
		
	}
	    std::string text = "in:" + std::to_string(event_infor.count_in);
	    std::string text1 = "out:" + std::to_string(event_infor.count_out);
        cv::putText(img, "in:" + std::to_string(event_infor.count_in), cv::Point(10,40),cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 255, 255), 1, 8, 0);
        cv::putText(img, "out:" + std::to_string(event_infor.count_out), cv::Point(10,80),cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 255, 255), 1, 8, 0);
        // count_to_clear.push_back(text);
        // count_to_clear.push_back(text1);
        // count_point_to_clear.push_back(cv::Point(10,40));
        // count_point_to_clear.push_back(cv::Point(10,80));
        // data_convert(ret,dec_data,&track_end_id,&size);
        count_to_clear[index].push_back(text);
        count_to_clear[index].push_back(text1);
        count_point_to_clear[index].push_back(cv::Point(10,40));
        count_point_to_clear[index].push_back(cv::Point(10,80));
	// line(img, Point(lines[0], lines[1]), Point(lines[2], lines[3]), cv::Scalar(0, 0, 255, 255), 2,CV_8S);
    // std::string text = "in:" + std::to_string(event_infor.count_in);
	// std::string text1 = "out:" + std::to_string(event_infor.count_out);
    // cv::putText(img, "in:" + std::to_string(event_infor.count_in), cv::Point(10,40),cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 255, 255), 1, 8, 0);
    // cv::putText(img, "out:" + std::to_string(event_infor.count_out), cv::Point(10,80),cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 255, 255), 1, 8, 0);
    // count_to_clear[index].push_back(text);
    // count_to_clear[index].push_back(text1);
    // count_point_to_clear[index].push_back(cv::Point(10,40));
    // count_point_to_clear[index].push_back(cv::Point(10,80));
}


CTraceCounter::CTraceCounter()
{
}

void CTraceCounter::run(cv::Mat vioPic, wi_rectangle* vipDetections,int* tracking_end_id, int vNumDetection,int size,int Runmode,int index)
{
	draw_trace(vioPic,index);
	generateTrace(vioPic,trace_infor, vipDetections, vNumDetection,tracking_end_id,size, Runmode);
	
	// cv::putText(vioPic, "No:" + std::to_string(1), cv::Point(HEAD_INPUT_IMG_W_H - 65, 45), 1, 3, cv::Scalar(255, 0, 255), 2);
}
