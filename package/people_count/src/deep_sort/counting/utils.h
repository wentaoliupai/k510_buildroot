///
/// @file
/// @copyright All code copyright Movidius Ltd 2012, all rights reserved.
///            For License Warranty see: common/license.txt
///
/// @brief     utils  Leon header
///

#ifndef _UTILS_H_
#define _UTILS_H_


#include <math.h>
#include <stdbool.h>
#include "types.h"

#define TraceNum 30
#define positionNum 500
#ifdef __cplusplus
extern "C" {
#endif

typedef struct wi_rectangle_t
{
    int      min_x;
    int      min_y;
    int      max_x;
    int      max_y;
    int      center_x;
    int      center_y;
    float    prob;
    int      trackid;
    int      traceid;
}wi_rectangle;

typedef struct wi_img_t {
    uint32_t            left;
    uint32_t            top;
    uint32_t            width;
    uint32_t            height;
    uint32_t            tsize;
    uint32_t            cnt;
    float               wiprob;
    uint32_t            trackid;
    uint64_t            timestamp;
}wi_img;

typedef struct
{
	uint64_t ts_usec;
	uint64_t chip_usec;
    int x;
    int y;
    int w;
    int h;
    float confidence;
} position;

typedef struct
{
    int flag_read_ready; 
    int flag_trace_over;
    int id;
    bool center_area;
    int count;
    bool cross_line;
	position positions[positionNum];
	int flag_read_over;
    int entry;
    int trackid;
    int traceid;
} traceInformation_io;

typedef struct face_detectinfo
{
    int  left;
    int  top;
    int  right;
    int  bottom;
    int  placeholder1;
    int  placeholder2;
}__attribute__((aligned(4)))detectinfo;

typedef struct wi_face_prob_info_t
{
    int yolo_prob;
    int blur_prob;
    int front_prob;
}wi_face_prob_info;

#ifdef __cplusplus
}
#endif

#endif
