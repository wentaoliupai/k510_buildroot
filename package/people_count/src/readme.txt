1.参数说明

" <kmodel> <net_len> <valid_width> <valid_height> <obj_thresh> <nms_thresh> <video_conf> <is_rgb> <enable_profile> <dump_image_dir> <iou_distance> <max_n_init>" 
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
    std::string dump_img_dir;
}ai_worker_args;	

kmodel：kmodel路径
net_len：模型网络宽度，目前使用yolov5,固定输入大小是320
valid_width：实际存储图像的宽度
valid_height: 实际存储图像的高度
obj_thresh：目标检测框阈值，默认0.5,值越小，越少的检测框被过滤掉
nms_thresh：nms阈值，默认0.45,值越小，越少的冗余检测框被过滤掉
video_conf：用于配置isp video5,valid_width和valid_height要与isp video5配置保持一致
is_rgb:isp图像是否RGB格式，生成的kmodel与之保持一致，默认是1
enable_profile：是否使用对各个模块进行计时，默认是0
dump_image_dir：预处理的图像的保存路径,默认是None,不进行保存
iou_distance：iou距离，距离越小说明两个框越匹配,默认是0.7
max_n_init：准轨迹转变为轨迹需要连续匹配的帧数，默认是3


使用示例：
./deep_sort yolov5s_320_sigmoid_bf16_with_preprocess_uint8_NHWC.kmodel 320 320 240 0.5 0.45 video_320x320_240.conf 1 0 None 0.7 3

2.接口说明
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