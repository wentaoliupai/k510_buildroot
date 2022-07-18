#include <nncase/runtime/host_runtime_tensor.h>
#include <nncase/runtime/interpreter.h>
#include <nncase/runtime/runtime_tensor.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/imgproc/types_c.h>
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
#include <semaphore.h>
#include <poll.h>
#include <atomic>
#include<vector>
#include<time.h> 
#include "k510_drm.h"
#include "media_ctl.h"
#include <linux/videodev2.h>

#include "VideoStreamer.hh"
#include "IRtspServer.h"
#include "enc_interface.h"

#include "object_detect.h"
#include "cv2_utils.h"
#include "deep_sort.h"
#include "definition/config.h"

#include "counting/traceCounter.h"
#include "counting/trace_gen_interface.h"
#include <linux/videodev2.h>
#include "v4l2.h"
struct video_info dev_info[2];

static int received_sigterm = 0;
#define SELECT_TIMEOUT		2000
std::mutex mtx;
uint8_t drm_bufs_index = 0;
uint8_t drm_bufs_argb_index = 0;
struct drm_buffer *fbuf_yuv, *fbuf_argb;
struct drm_buffer *fbuf_yuv_clear, *fbuf_argb_clear;
int obj_cnt;
#define MODE 2
std::vector<cv::Point> points_to_clear;
std::vector<std::string> strs_to_clear;
std::vector<std::string> count_to_clear;
std::vector<cv::Point> count_point_to_clear;
std::vector<traceInformation_io> track_to_clear;
std::atomic<bool> quit(true);
EncoderHandle *pCfg;
IRtspServerEX *pRtspServer;

extern stru_event_infor event_infor;
// #define is_rtsp
// #define LOG_ON 1
void fun_sig(int sig)
{
    if(sig == SIGINT)
    {
        quit.store(false);
    }
}


static unsigned long int get_time()
{
  struct timespec time;
        
  clock_gettime(CLOCK_REALTIME, &time);
  
  return time.tv_sec * 1000LL * 1000LL * 1000LL + time.tv_nsec;
}

void ai_rgb_worker(ai_worker_args ai_args)
{
    /****fixed operation for video operation****/
    mtx.lock();
    cv::VideoCapture capture;
    capture.open(5);
    // video setting
    capture.set(cv::CAP_PROP_CONVERT_RGB, 0);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, ai_args.net_len);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, ai_args.net_len);
    // RRRRRR....GGGGGGG....BBBBBB, CHW
    capture.set(cv::CAP_PROP_FOURCC, V4L2_PIX_FMT_RGB24);
    mtx.unlock();

    DeepSort ds(ai_args);
    int frame_cnt = 0;

    // define cv::Mat for ai input
    // padding offset is (valid_width - valid_height) / 2 * valid_width
    cv::Mat rgb24_img_for_ai(ai_args.net_len, ai_args.net_len, CV_8UC3, ds.get_det()->virtual_addr_input[0] + (ai_args.valid_width - ai_args.valid_height) / 2 * ai_args.valid_width);
    tracker mytracker(ai_args.iou_distance, ai_args.max_n_init);
    while (quit.load())
    {
        //ScopedTiming st("total", 1);
        mtx.lock();
        capture.read(rgb24_img_for_ai);
        mtx.unlock();
        
        DEEP_SORT_RET ret;
        cv::Mat img = cv::imread("./000001.jpg");
        ds.run_with_det_model(img, ret);

        //对于result的操作

        /****fixed operation for display clear****/
        cv::Mat img_argb;
        {
            ScopedTiming st("display clear", ai_args.enable_profile);
            fbuf_argb = &drm_dev.drm_bufs_argb[drm_bufs_argb_index];
            img_argb = cv::Mat(DRM_INPUT_HEIGHT, DRM_INPUT_WIDTH, CV_8UC4, (uint8_t*)fbuf_argb->map);
            fbuf_argb_clear = &drm_dev.drm_bufs_argb[!drm_bufs_argb_index];
            cv::Mat img_argb_clear = cv::Mat(DRM_INPUT_HEIGHT, DRM_INPUT_WIDTH, CV_8UC4, (uint8_t *)fbuf_argb_clear->map);
            // img_argb.setTo(cv::Scalar(0, 0, 0, 0));

            for (uint32_t i = 0; i < obj_cnt; i++)
            {
                struct vo_draw_frame frame;
                frame.crtc_id = drm_dev.crtc_id;
                frame.draw_en = 0;
                frame.frame_num = i;
                draw_frame(&frame);
            }
        }

        {
            ScopedTiming st("draw osd", ai_args.enable_profile);
            obj_cnt = 0;

            for (Track& track : ds.get_tracker()->tracks)
            {
                //py版本是>1,有可能出现只显示预测框，不显示检测框的情况
                //if (!track.is_confirmed() || track.time_since_update > 1)
                if (!track.is_confirmed() || track.time_since_update >= 1)
                    continue;
                if (obj_cnt < 32)
                {
                    struct vo_draw_frame frame;
                    frame.crtc_id = drm_dev.crtc_id;
                    frame.draw_en = 1;
                    frame.frame_num = obj_cnt;
                    DETECTBOX tmp = track.to_tlwh();
                    std::cout << tmp << std::endl;
                    frame.line_x_start = tmp[0]*DRM_INPUT_WIDTH/ai_args.valid_width;
                    frame.line_y_start = tmp[1]*DRM_INPUT_HEIGHT/ai_args.valid_width+DRM_OFFSET_HEIGHT;
                    frame.line_y_start = frame.line_y_start < DRM_OFFSET_HEIGHT ? DRM_OFFSET_HEIGHT : frame.line_y_start;
                    frame.line_x_end = (tmp[0]+tmp[2])*DRM_INPUT_WIDTH/ai_args.valid_height;
                    frame.line_y_end = (tmp[1]+tmp[3])*DRM_INPUT_HEIGHT/ai_args.valid_height+DRM_OFFSET_HEIGHT;
                    frame.line_y_end = frame.line_y_end > (DRM_OFFSET_HEIGHT+DRM_INPUT_HEIGHT) ? (DRM_OFFSET_HEIGHT+DRM_INPUT_HEIGHT) : frame.line_y_end;
                    draw_frame(&frame);

                    cv::Point origin;
                    origin.x = (int)(tmp[0] * DRM_INPUT_WIDTH / ai_args.valid_width);
                    origin.y = (int)(tmp[1] * DRM_INPUT_HEIGHT / ai_args.valid_height + 10);
                    std::string text = std::to_string(track.track_id);
                    cv::putText(img_argb, text, origin, cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 255, 255), 1, 8, 0);

                }
                obj_cnt += 1;
            }
        }
        frame_cnt += 1;
        drm_bufs_argb_index = !drm_bufs_argb_index;
    }

    /****fixed operation for capture release and display clear****/
    printf("%s ==========release \n", __func__);
    mtx.lock();
    capture.release();
    mtx.unlock();
    for (uint32_t i = 0; i < obj_cnt; i++)
    {
        struct vo_draw_frame frame;
        frame.crtc_id = drm_dev.crtc_id;
        frame.draw_en = 0;
        frame.frame_num = i;
        draw_frame(&frame);
    }
}



void data_convert(DEEP_SORT_RET ret,wi_rectangle *dec_data,vector<int> *dis_id,int *rect_size)
{
    vector<DEEP_SORT_TRACK_RET>::iterator it;
    vector<int>::iterator end_id;
    int i = 0;
    for(it=ret.tracks_ret.begin();it !=ret.tracks_ret.end();it++)
    {
        dec_data[i].trackid = it->track_id;
        dec_data[i].min_x = (it->track_bbox[0])*DRM_INPUT_WIDTH/320;
        dec_data[i].min_y = (it->track_bbox[1])*DRM_INPUT_HEIGHT/240;
        dec_data[i].max_x= (it->track_bbox[2] + it->track_bbox[0])*DRM_INPUT_WIDTH/320;
        dec_data[i].max_y = (it->track_bbox[3] + it->track_bbox[1])*DRM_INPUT_HEIGHT/240;
        
        dec_data[i].center_x = (dec_data[i].max_x + dec_data[i].min_x)/2;
        dec_data[i].center_y = (dec_data[i].max_y + dec_data[i].min_y)/2;
        i++;
    }
    (*dis_id).assign(ret.disp_track_id.begin(),ret.disp_track_id.end());
    // if(ret.disp_track_id.size() != 0 || (*dis_id).size() != 0)
    //     printf("disp_id_size : %d  %d\n",ret.disp_track_id.size(),(*dis_id).size());
    *rect_size = i;
    
}

void ai_worker(ai_worker_args ai_args)
{    
    /****fixed operation for video operation****/
    mtx.lock();
    cv::VideoCapture capture;
	  capture.open(5);
    // video setting
    capture.set(cv::CAP_PROP_CONVERT_RGB, 0);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, ai_args.net_len);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, ai_args.net_len);
    // RRRRRR....GGGGGGG....BBBBBB, CHW
    capture.set(cv::CAP_PROP_FOURCC, V4L2_PIX_FMT_RGB24);
    mtx.unlock();
    cv::Mat yuvImage;

    cv::Mat img_argb_clear;
    vector<int> track_end_id;
    int vNumDetection = 0;
    int track_size = 0;
    int frame_cnt = 0;
    wi_rectangle *dec_data = (wi_rectangle*)malloc(sizeof(wi_rectangle) * 30);
    static int size = 0;
    DeepSort ds(ai_args);
    // define cv::Mat for ai input
    // padding offset is (valid_width - valid_height) / 2 * valid_width
    cv::Mat rgb24_img_for_ai(ai_args.net_len, ai_args.net_len, CV_8UC3, ds.get_det()->virtual_addr_input[0] + (ai_args.valid_width - ai_args.valid_height) / 2 * ai_args.valid_width);
    // cv::Mat osd_img;

    //tracker mytracker(ai_args.iou_distance,ai_args.max_n_init);
    while(quit.load()) 
    {
        //ScopedTiming st("total", 1);
        mtx.lock();
        capture.read(rgb24_img_for_ai);
        mtx.unlock();         
        DEEP_SORT_RET ret;
        ds.run_with_det_model_isp(ret);
#ifdef LOG_ON       
        std::cout << "for disp_track_id:" << std::endl;
        for(auto iter = ret.disp_track_id.begin();iter!=ret.disp_track_id.end();++iter)
        {
            std::cout<<*iter<<std::endl;
        }
        std::cout << "for tracks_ret:" << std::endl;
        for(auto iter=ret.tracks_ret.begin();iter!=ret.tracks_ret.end();++iter)
        {
            std::cout<<"track_id:"<<iter->track_id<<" track_bbox:"<<iter->track_bbox<<std::endl;
        }
#endif
        static CTraceCounter* pTraceCounter = CTraceCounter::getOrCreateInstance();
        
        /****fixed operation for display clear****/
        cv::Mat img_argb;
        {
            ScopedTiming st("display clear", ai_args.enable_profile);
            fbuf_argb = &drm_dev.drm_bufs_argb[drm_bufs_argb_index];
            img_argb = cv::Mat(DRM_INPUT_HEIGHT, DRM_INPUT_WIDTH, CV_8UC4, (uint8_t *)fbuf_argb->map);
            fbuf_argb_clear = &drm_dev.drm_bufs_argb[!drm_bufs_argb_index];
            img_argb_clear = cv::Mat(DRM_INPUT_HEIGHT, DRM_INPUT_WIDTH, CV_8UC4, (uint8_t *)fbuf_argb_clear->map);
            // img_argb.setTo(cv::Scalar(0,0,0,0));
            for (uint32_t cc = 0; cc < points_to_clear.size(); cc++)
            {          
                cv::putText(img_argb_clear, strs_to_clear[cc], points_to_clear[cc], cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 0, 0), 1, 8, 0);
            }
            for (uint32_t cc1 = 0; cc1 < count_to_clear.size(); cc1++)
            {          
                cv::putText(img_argb_clear, count_to_clear[cc1], count_point_to_clear[cc1], cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 0, 0), 1, 8, 0);
            }
            for (traceInformation_io& track : track_to_clear)
            {          
            	if (track.count > 0)
		        {			
                    for (int j = 0; j < track.count - 1; j++)
                    {
                        
                        cv::line(img_argb_clear, cv::Point(track.positions[j].x, track.positions[j].y), cv::Point(track.positions[j + 1].x, track.positions[j + 1].y), cv::Scalar(0, 0, 0, 0),2,CV_8S);
                        cv::circle(img_argb_clear, cv::Point(track.positions[j + 1].x, track.positions[j + 1].y), 3,cv::Scalar(0, 0, 0, 0), -1);
                        
                    }
		        }
            }
            for(uint32_t i = 0; i < obj_cnt; i++)
            {
                struct vo_draw_frame frame;
                frame.crtc_id = drm_dev.crtc_id;
                frame.draw_en = 0;
                frame.frame_num = i;
                draw_frame(&frame);
            }
            
        }
        count_to_clear.clear();
        count_point_to_clear.clear();
        track_to_clear.clear();
        data_convert(ret,dec_data,&track_end_id,&size);
        std::string text = "in:" + std::to_string(event_infor.count_in);
	    std::string text1 = "out:" + std::to_string(event_infor.count_out);
        cv::putText(img_argb, "in:" + std::to_string(event_infor.count_in), cv::Point(10,40),cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 255, 255), 1, 8, 0);
        cv::putText(img_argb, "out:" + std::to_string(event_infor.count_out), cv::Point(10,80),cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 255, 255), 1, 8, 0);
        count_to_clear.push_back(text);
        count_to_clear.push_back(text1);
        count_point_to_clear.push_back(cv::Point(10,40));
        count_point_to_clear.push_back(cv::Point(10,80));

        pTraceCounter->run(img_argb,dec_data,track_end_id.data(),size,track_end_id.size(),MODE);
        {
            ScopedTiming st("draw osd", ai_args.enable_profile);
            cv::Mat img_argb;        
            fbuf_argb = &drm_dev.drm_bufs_argb[drm_bufs_argb_index];
            img_argb = cv::Mat(DRM_INPUT_HEIGHT, DRM_INPUT_WIDTH, CV_8UC4, (uint8_t *)fbuf_argb->map);
            obj_cnt = 0;
            points_to_clear.clear();
            strs_to_clear.clear();
            for (Track& track : ds.get_tracker()->tracks)
            {
                //py版本是>1,有可能出现只显示预测框，不显示检测框的情况
                //if (!track.is_confirmed() || track.time_since_update > 1)
                if (!track.is_confirmed() || track.time_since_update >= 1)
                    continue;
                if (obj_cnt < 32)
                {
                    struct vo_draw_frame frame;
                    frame.crtc_id = drm_dev.crtc_id;
                    frame.draw_en = 1;
                    frame.frame_num = obj_cnt;
                    DETECTBOX tmp = track.to_tlwh();
                    
                    int x0  = tmp[0]*DRM_INPUT_WIDTH/ai_args.valid_width;
                    int y0  = tmp[1]*DRM_INPUT_HEIGHT/ai_args.valid_height;
                    int x1  = (tmp[0]+tmp[2])*DRM_INPUT_WIDTH/ai_args.valid_width;
                    int y1  = (tmp[1]+tmp[3])*DRM_INPUT_HEIGHT/ai_args.valid_height;
                    x1 = std::max(0, std::min(x1, DRM_INPUT_WIDTH));
                    x0 = std::max(0, std::min(x0, DRM_INPUT_WIDTH));
                    y0 = std::max(0, std::min(y0, DRM_INPUT_HEIGHT));
                    y1 = std::max(0, std::min(y1, DRM_INPUT_HEIGHT));
                    frame.line_x_start = x0;
                    frame.line_x_end = x1;
                    frame.line_y_start = y0 + DRM_OFFSET_HEIGHT;
                    frame.line_y_end = y1 + DRM_OFFSET_HEIGHT;
                    draw_frame(&frame);
                
                    cv::Point origin;
                    origin.x = (int)(tmp[0] * DRM_INPUT_WIDTH / ai_args.valid_width);
                    origin.y = (int)(tmp[1] * DRM_INPUT_HEIGHT / ai_args.valid_height + 10);
                    std::string text = std::to_string(track.track_id);
                    cv::putText(img_argb, text, origin, cv::FONT_HERSHEY_COMPLEX, 1.5, cv::Scalar(0, 0, 255, 255), 1, 8, 0);
                    points_to_clear.push_back(origin);
                    strs_to_clear.push_back(text);
                }
                    // line(img_argb, cv::Point(DRM_INPUT_WIDTH/2,0), cv::Point(DRM_INPUT_WIDTH/2,DRM_INPUT_HEIGHT), cv::Scalar(0, 0, 255, 255), 2,CV_8S);
      
                obj_cnt += 1;
            }
        }
        frame_cnt += 1;
        drm_bufs_argb_index = !drm_bufs_argb_index;

        
    }
    
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

static int video_stop(struct v4l2_device *vdev)
{
	int ret;

	ret = v4l2_stream_off(vdev);
	if (ret < 0) {
		printf("error: failed to stop video stream: %s (%d)\n",
			strerror(-ret), ret);
		return ret;
	}

	return 0;
}

static void video_cleanup(struct v4l2_device *vdev)
{
	if (vdev) {
		v4l2_free_buffers(vdev);
		v4l2_close(vdev);
	}
}
static int process_ds0_image(struct v4l2_device *vdev,unsigned int width,unsigned int height)
{
	struct v4l2_video_buffer buffer;
	int ret;
    static struct v4l2_video_buffer old_buffer;
    static int screen_init_flag = 0;

    mtx.lock();
	ret = v4l2_dequeue_buffer(vdev, &buffer);
	if (ret < 0) {
		printf("error: unable to dequeue buffer: %s (%d)\n",
			strerror(-ret), ret);
        mtx.unlock();
		return ret;
	}
    mtx.unlock();
	if (buffer.error) {
		printf("warning: error in dequeued buffer, skipping\n");
		return 0;
	}

    fbuf_yuv = &drm_dev.drm_bufs[buffer.index];

    if (drm_dev.req)
        drm_wait_vsync();
    fbuf_argb = &drm_dev.drm_bufs_argb[!drm_bufs_argb_index];
    if (drm_dmabuf_set_plane(fbuf_yuv, fbuf_argb)) {
        std::cerr << "Flush fail \n";
        return 1;
    }

    if(screen_init_flag) {
        fbuf_yuv = &drm_dev.drm_bufs[old_buffer.index];
        old_buffer.mem = fbuf_yuv->map;
        old_buffer.size = fbuf_yuv->size;
        mtx.lock();
        ret = v4l2_queue_buffer(vdev, &old_buffer);
        if (ret < 0) {
            printf("error: unable to requeue buffer: %s (%d)\n",
                strerror(-ret), ret);
            mtx.unlock();
            return ret;
        }
        mtx.unlock();
    }
    else {
        screen_init_flag = 1;
    }

    old_buffer = buffer;

	return 0;
}


void display_worker(int enable_profile)
{
    int ret;
    struct v4l2_device *vdev;
    struct v4l2_pix_format format;
    fd_set fds;
    struct v4l2_video_buffer buffer;
	unsigned int i;

    mtx.lock();
    vdev = v4l2_open(dev_info[0].video_name[1]);
    if (vdev == NULL) {
		printf("error: unable to open video capture device %s\n",
			dev_info[0].video_name[1]);
        mtx.unlock();
		goto display_cleanup;
	}

	memset(&format, 0, sizeof format);
	format.pixelformat = dev_info[0].video_out_format[1] ? V4L2_PIX_FMT_NV12 : V4L2_PIX_FMT_NV16;
	format.width = dev_info[0].video_width[1];
	format.height = dev_info[0].video_height[1];

	ret = v4l2_set_format(vdev, &format);
	if (ret < 0)
	{
		printf("%s:v4l2_set_format error\n",__func__);
        mtx.unlock();
		goto display_cleanup;
	}

	ret = v4l2_alloc_buffers(vdev, V4L2_MEMORY_USERPTR, DRM_BUFFERS_COUNT);
	if (ret < 0)
	{
		printf("%s:v4l2_alloc_buffers error\n",__func__);
        mtx.unlock();
		goto display_cleanup;
	}

	FD_ZERO(&fds);
	FD_SET(vdev->fd, &fds);

	for (i = 0; i < vdev->nbufs; ++i) {
		buffer.index = i;
        fbuf_yuv = &drm_dev.drm_bufs[buffer.index];
        buffer.mem = fbuf_yuv->map;
        buffer.size = fbuf_yuv->size;

		ret = v4l2_queue_buffer(vdev, &buffer);
		if (ret < 0) {
			printf("error: unable to queue buffer %u\n", i);
            mtx.unlock();
			goto display_cleanup;
		}	
	}

	ret = v4l2_stream_on(vdev);
	if (ret < 0) {
		printf("%s error: failed to start video stream: %s (%d)\n", __func__,
			strerror(-ret), ret);
        mtx.unlock();
		goto display_cleanup;
	}
    mtx.unlock();

    while(quit.load()) {
		struct timeval timeout;
		fd_set rfds;

		timeout.tv_sec = SELECT_TIMEOUT / 1000;
		timeout.tv_usec = (SELECT_TIMEOUT % 1000) * 1000;
		rfds = fds;

		ret = select(vdev->fd + 1, &rfds, NULL, NULL, &timeout);
		if (ret < 0) {
			if (errno == EINTR)
				continue;

			printf("error: select failed with %d\n", errno);
			goto display_cleanup;
		}
		if (ret == 0) {
			printf("error: select timeout\n");
			goto display_cleanup;
		}
        process_ds0_image(vdev, format.width, format.height);
    }

display_cleanup:
    mtx.lock();
    video_stop(vdev);
	video_cleanup(vdev);
    mtx.unlock();
}

int main(int argc, char *argv[])
{
    //std::cout << "case " << argv[0] << " build " << __DATE__ << " " << __TIME__ << std::endl;
    if (argc < 11)
    {
        std::cerr << "Object detect by YOLOV5 Usage: " << argv[0] 
            << " <kmodel> <net_len> <valid_width> <valid_height> <obj_thresh> <nms_thresh> <video_conf> <is_rgb> <enable_profile> <dump_image_dir> <iou_distance> <max_n_init> <max_hits>" << std::endl;
        return -1;
    }
    // parse args for ai worker
    ai_worker_args ai_args;
    ai_args.kmodel_path = argv[1];
    ai_args.net_len = atoi(argv[2]);
    ai_args.valid_width = atoi(argv[3]);
    ai_args.valid_height = atoi(argv[4]);
    if(ai_args.valid_height > ai_args.valid_width)
    {
        std::cerr << "You should set width bigger than height" << std::endl;
                std::abort();
    }
    if(ai_args.valid_width != ai_args.net_len)
    {
        std::cerr << "We won't resize image for gnne input, so valid_width should be equal to net_len" << std::endl;
                std::abort();
    }
    ai_args.obj_thresh = atof(argv[5]);
    ai_args.nms_thresh = atof(argv[6]);
    char* video_cfg_file = argv[7];
    ai_args.is_rgb = atoi(argv[8]);
    ai_args.enable_profile = atoi(argv[9]);
    int enable_profile = atoi(argv[9]);
    ai_args.dump_img_dir = argv[10];

    ai_args.iou_distance = atof(argv[11]);
    ai_args.max_n_init = atoi(argv[12]);
    ai_args.max_hits = atoi(argv[13]);

    /****fixed operation for ctrl+c****/
    struct sigaction sa;
    memset( &sa, 0, sizeof(sa) );
    sa.sa_handler = fun_sig;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);
 
    if(drm_init())
    {
        return -1;
    }
    mediactl_init(video_cfg_file, &dev_info[0]);
    std::thread thread_ds2(ai_worker, ai_args);
    std::thread thread_ds0(display_worker, enable_profile);

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

    mediactl_exit();
    return 0;
}