#ifndef trace_gen_interface_h__
#define trace_gen_interface_h__

#include <stdint.h>
#include "utils.h"

#define boxNum 20   //max num of detected boxed in  frame
#define delay_frame 10

#define HEAD_INPUT_IMG_W_H 512    //input image width and height
#define ELAPSE_PER_FRAME_MS 146

#define RATIO_SPACE_COLOR  0.5   //the threshold    place distance divide color distance
#define THRESHOLD_SPACE_DIST  ( 66 / 288.0 )   //space threshold
#define THRESHOLD_COLOR_DIST 0.22               //color threshold
#define THRESHOLD_TOTAL_DIST   (THRESHOLD_SPACE_DIST* RATIO_SPACE_COLOR+(1-RATIO_SPACE_COLOR)*THRESHOLD_COLOR_DIST *3) //the distance threshold used for determine the box belongs to which trajectory

#define BACKGROUND_THRESHOLD 15    //extract foreground threshold
#define NUM_TIMESTAMP 100
#define TraceNum 30
#define positionNum 500

typedef struct
{
	int count_in; //
	uint64_t in_event[NUM_TIMESTAMP];
	int count_out; //
	uint64_t out_event[NUM_TIMESTAMP];
} stru_event_infor;

extern stru_event_infor event_infor;

int generateTrace(traceInformation_io* vpTraceSet, wi_rectangle* vipDetections, int vNumDetection, int* tracking_end_id,int size, char vRUN_MODE);

//trajectories storage zone garbage collection
int gc_trace_infor(traceInformation_io* _trace_infor);
 

//add new box to the one trajectory 
int addBox2TraceAndCounting(traceInformation_io *vpTraces, position _box, int vTraceIndex,wi_rectangle *vpDetection);

int allocation_trace_mem_id(traceInformation_io* _trace_infor,int id  ,int flag );
int jude_trace_over_valid_set(traceInformation_io* _trace_infor,int* tracking_end_id,int size);

#endif
