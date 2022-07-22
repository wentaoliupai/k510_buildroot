#include <string.h>
#include <arpa/inet.h>
#include "trace_gen_interface.h"
#include <vector>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include "stdlib.h"
#include "stdio.h"



#define TraceNum 30
#define LINEMODE 1
#define CENTERIGNORE 1
#define WIDTH 1080
#define HEIGHT 1920
#define positionNum_inout 128

#define max(a,b) ( ((a)>(b)) ? (a):(b) )
#define min(a,b) ( ((a)>(b)) ? (b):(a) )

stru_event_infor event_infor = {0};
// int cross_line[40] = {0, HEIGHT/2, WIDTH, HEIGHT/2}; //横线
int cross_line[40] = {WIDTH/2, 0, WIDTH/2, HEIGHT}; //竖线

int center_area[4] = {WIDTH / 4, HEIGHT / 4, WIDTH - WIDTH / 4, HEIGHT - HEIGHT / 4};
int cross_line_pts_num = 2;
long dis_threash = HEIGHT * HEIGHT / 16;
int set_cross_line[40] = {0};
int set_line_lenth = 0;
int last_end_id[10] = {0};



uint64_t wiGetSysTimestamps(void)
{
    time_t tNow = 0;
    time(&tNow);
    return tNow;
}

void _convert(wi_rectangle *vpDetection, position &voBox) {
    
    voBox.x = (vpDetection->max_x + vpDetection->min_x) / 2;
    voBox.y = (vpDetection->max_y + vpDetection->min_y) / 2;
    voBox.w = vpDetection->max_x - vpDetection->min_x;
    voBox.h = vpDetection->max_y - vpDetection->min_y;
    voBox.chip_usec = wiGetSysTimestamps();
    voBox.ts_usec = wiGetSysTimestamps();
    voBox.confidence = vpDetection->prob;
}

void jduge_line_mode(void)
{
	if(set_line_lenth != 0)
	{
		cross_line_pts_num = set_line_lenth;
		for(int num = 0;num < cross_line_pts_num;num++)
		{
			cross_line[num] = htonl(set_cross_line[num]);
		}
		set_line_lenth = 0;
	}

	return;
}



bool _tooFar(position &vLatestPos, wi_rectangle *vpBoxDetected) {
#define MaxWidthDiff 600
#define MaxHeightDif 450
    if (abs(vLatestPos.x - (vpBoxDetected->max_x + vpBoxDetected->min_x) / 2) > MaxWidthDiff)
        return true;
    if (abs(vLatestPos.y - (vpBoxDetected->max_y + vpBoxDetected->min_y) / 2) > MaxHeightDif)
        return true;
    return false;
}

void whichTraceBelongTo(traceInformation_io *vpTraceSet, wi_rectangle *vpBoxesDetected, int vNumDetection) {
    for (int k, i = 0; i < vNumDetection; ++i) {
        int trackid = vpBoxesDetected->trackid;
        for (k = 0; k < TraceNum; k++) {
            traceInformation_io &trace = *(vpTraceSet + k);
            if ((trace.trackid == trackid) && (trace.count > 0))
            {
                vpBoxesDetected->traceid = trace.traceid;
                break;
            }
        }
        if (k == TraceNum)
        {
            vpBoxesDetected->traceid = (-1 - vpBoxesDetected->traceid) >= 0 ? (-31 + vpBoxesDetected->traceid) : (-1 - vpBoxesDetected->traceid);
        }
            
        
        vpBoxesDetected++;
    }
}

int getOrCreatTraceIndex(traceInformation_io *vpTraceSet, wi_rectangle *vpDetection, char vRUN_MODE) {
    int traceIndex = vpDetection->traceid;
    if (traceIndex < 0) //this box not belong to current trajectory
    {
        traceIndex = allocation_trace_mem_id(vpTraceSet, -1, 0);
        vpTraceSet[traceIndex].traceid = traceIndex;
    }
    return traceIndex;
}

int generateTrace(cv::Mat vioPic,traceInformation_io *vpTraceSet, wi_rectangle *vpDetections, int vNumDetection,
		int* tracking_end_id , int size,char vRUN_MODE) {
	// jduge_line_mode();
    int flag = 1;
	jude_trace_over_valid_set(vpTraceSet,tracking_end_id,size);
	//if(vRUN_MODE == 16)
	{
		gc_trace_infor(vioPic,vpTraceSet);
	}
    if (vNumDetection == 0)return 1;
    whichTraceBelongTo(vpTraceSet, vpDetections, vNumDetection);
    for (int i = 0; i < vNumDetection; ++i) {
    	for(int k = 0;k < size;k++)
    	{
    		if(vpDetections->trackid == last_end_id[k])
    		{
    			flag = 0;
    			break;
    		}
    	}
    	if(flag == 0)
    	{
    		flag = 1;
    	}
    	else
    	{
            static position pos;
            _convert(vpDetections, pos);
            int traceIndex = getOrCreatTraceIndex(vpTraceSet, vpDetections, vRUN_MODE);
            addBox2TraceAndCounting(vpTraceSet, pos, traceIndex,vpDetections);
            vpDetections++;
    	}

    }

    return 0;
}

// trajectories storage zone garbage collection
int gc_trace_infor(cv::Mat vioPic,traceInformation_io *vpTraceSet) {
    for (int i = 0; i < TraceNum; ++i) {
        if (vpTraceSet->flag_read_over == 1) {
            memset(vpTraceSet, 0, sizeof(traceInformation_io));
        }
        vpTraceSet++;
    }
    return 0;
}




bool is_cross(int a0, int a1, int b0, int b1, int c0, int c1, int d0, int d1) {
    return (min(a0, b0) <= max(c0, d0) && min(c1, d1) <= max(a1, b1) && min(c0, d0) <= max(a0, b0) &&
            min(a1, b1) <= max(c1, d1));
}

bool is_in_area(int area1x, int area1y, int area2x, int area2y, int pt1_x, int pt1_y) {
    return !(area1x < pt1_x && area1y < pt1_y && area2x > pt1_x && area2y > pt1_y);
}

int cal_angle(int x1, int y1, int x2, int y2) {
    int delta_x = x1 - x2;
    int delta_y = y1 - y2;
    double angle_cos = delta_y * 1.0 / sqrt(double(delta_x * delta_x + delta_y * delta_y));
    double ret_angle = acos(angle_cos) * 180.0 / M_PI;
    return ret_angle;
}

double angle_thresh = 85.0;

int addBox2TraceAndCounting(traceInformation_io *vpTraces, position _box, int vTraceIndex,wi_rectangle *vpDetection) {
    vpTraces += vTraceIndex;
    if (vpTraces->count == positionNum) {
        int numReserve = positionNum / 2 - 1;
        position *p = vpTraces->positions;
        memcpy(p + 1, p + positionNum - numReserve, sizeof(position) * numReserve);
        vpTraces->count = numReserve + 1;
    }
    int index = vpTraces->count;
    vpTraces->trackid = vpDetection->trackid;
    vpTraces->count += 1;
    vpTraces->positions[index].ts_usec = _box.ts_usec;
    vpTraces->positions[index].chip_usec = _box.chip_usec;
    vpTraces->positions[index].x = _box.x;
    vpTraces->positions[index].y = _box.y;
    vpTraces->positions[index].w = _box.w;
    vpTraces->positions[index].h = _box.h;
    vpTraces->positions[index].confidence = _box.confidence;
    vpTraces->center_area = true;
    if (vpTraces->count > 1) {
        if (LINEMODE) {
            int index = vpTraces->count - 1;
            vpTraces->cross_line = false;
            for (int idx = 0; idx <= cross_line_pts_num - 2; idx += 1) {

                if (is_cross(cross_line[2 * idx], cross_line[2 * idx + 1], cross_line[2 * idx + 2],
                             cross_line[2 * idx + 3], vpTraces->positions[index-1].x, vpTraces->positions[index-1].y,
                             vpTraces->positions[index].x, vpTraces->positions[index].y)) {        
                    vpTraces->cross_line = true;
                    break;
                }
            }
        }
    }


    return 0;
}



int Counting(traceInformation_io *vpTraces) {

    int fp_index = 0;
    int lp_index = vpTraces->count - 1;
    int start_point_x = vpTraces->positions[fp_index].x;
    int start_point_y = vpTraces->positions[fp_index].y;
    int end_point_x = vpTraces->positions[lp_index].x;
    int end_point_y = vpTraces->positions[lp_index].y;
    double angle = cal_angle(end_point_x, end_point_y, start_point_x, start_point_y);
    long distance = (start_point_y - end_point_y) * (start_point_y - end_point_y) +
                    (start_point_x - end_point_x) * (start_point_x - end_point_x);

    if (vpTraces->count > 1) {
        if (LINEMODE){
            for (int idx = 0; idx <= cross_line_pts_num - 2; idx += 1) {
                if (is_cross(cross_line[2 * idx], cross_line[2 * idx + 1], cross_line[2 * idx + 2],
                		cross_line[2 * idx + 3], start_point_x, start_point_y,
						end_point_x, end_point_y)) {
                    vpTraces->cross_line = true;
                    break;
                }
            }
        }
    }
    uint64_t Trace_pc_s = wiGetSysTimestamps();
    //if ((vpTraces->cross_line) && (vpTraces->center_area) && (distance > dis_threash)) {
        if ((vpTraces->cross_line) && (vpTraces->center_area)) {
        if (end_point_x >= start_point_x) {
            event_infor.in_event[event_infor.count_in % NUM_TIMESTAMP] = Trace_pc_s;
            event_infor.count_in++;
            vpTraces->entry = 1;
            //printf("Camera ---------------------- event_infor.count_in ++, vpTraces->id = %d  timestamp = %lld \n",vpTraces->trackid, Trace_pc_s);
        } else if (end_point_x < start_point_x) {
            event_infor.out_event[event_infor.count_out % NUM_TIMESTAMP] = Trace_pc_s;
            event_infor.count_out++;
            vpTraces->entry = 2;
            //printf("Camera ---------------------- event_infor.count_out ++, vpTraces->id = %d  timestamp = %lld \n",vpTraces->trackid, Trace_pc_s);
        } else
        {
        	vpTraces->entry = 0;
        	//printf("0----------------------------vpTraces->id = %d  point_count = %d \n",vpTraces->trackid, vpTraces->count);
        }

    } else
    {
    	vpTraces->entry = 3;
    	//printf("3-----------------------------------vpTraces->id = %d  point_count = %d \n",vpTraces->trackid, vpTraces->count);
    }

    vpTraces->flag_read_over = 1;
    return 0;
}

int allocation_trace_mem_id(traceInformation_io *vpTraceSet, int id, int flag) {
    int dstIndex = 0;
    for (; dstIndex < TraceNum; ++dstIndex)
        if ((vpTraceSet + dstIndex)->count == 0)
            break;

    if (dstIndex == TraceNum)
    {
        uint64_t timestamp_t = vpTraceSet[0].positions[vpTraceSet[0].count - 1].chip_usec;
        for (int i = 1; i < TraceNum; ++i) {
            uint64_t tmp = vpTraceSet[i].positions[(vpTraceSet[i].count - 1) % positionNum].chip_usec;
            if (tmp < timestamp_t) {
                timestamp_t = tmp;
                dstIndex = i;
            }
        }
    }
    memset(vpTraceSet + dstIndex, 0, sizeof(traceInformation_io));
    
    return dstIndex;
}


int jude_trace_over_valid_set(traceInformation_io *vpTraceSet,int* tracking_end_id,int size) {
	int tracking_end = 0;
	for(int t = 0;t < size;t++)
    {

		tracking_end = tracking_end_id[t];
	    last_end_id[t] = tracking_end_id[t];
    	for (int i = 0; i < TraceNum; i++)
    	{
    		if(vpTraceSet[i].trackid == tracking_end)
    		{
    			Counting(vpTraceSet+i);
                break;
    		}
    	}
    }
    return 0;
}


