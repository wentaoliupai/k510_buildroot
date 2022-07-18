#pragma once

#include "Singleton.h"
#include "trace_gen_interface.h"
//namespace cv{class Mat;}
class CTraceCounter : public CSingleton<CTraceCounter>
{
public:
	virtual ~CTraceCounter() {}
	void run(cv::Mat vioPic, wi_rectangle* vipDetections,int* tracking_end_id, int vNumDetection,int size,int Runmode);

private:
	CTraceCounter();
	friend class CSingleton<CTraceCounter>;
};
