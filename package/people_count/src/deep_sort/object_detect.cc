#include "object_detect.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

template <class T>
std::vector<T> read_binary_file(const char *file_name)
{
    std::ifstream ifs(file_name, std::ios::binary);
    ifs.seekg(0, ifs.end);
    size_t len = ifs.tellg();
    std::vector<T> vec(len / sizeof(T), 0);
    ifs.seekg(0, ifs.beg);
    ifs.read(reinterpret_cast<char *>(vec.data()), len);
    ifs.close();
    return vec;
}
int r1_count = 0;
void objectDetect::prepare_memory()
{
    input_size = ((net_len * net_len * (INPUT_CHANNELS + 1) + 4095) & (~4095));
    output_size[0] = (FEATURE_LEN * (net_len / 8)  * (net_len / 8)  * sizeof(float));
    output_size[1] = (FEATURE_LEN * (net_len / 16)  * (net_len / 16)  * sizeof(float));
    output_size[2] = (FEATURE_LEN * (net_len / 32)  * (net_len / 32)  * sizeof(float));
    output_all_size = ((output_size[0] + output_size[1] + output_size[2] + 4095) & (~4095));

    allocAlignMemOdOutput.size = output_all_size;
    allocAlignMemOdOutput.alignment = MEMORY_TEST_BLOCK_ALIGN;
    allocAlignMemOdOutput.phyAddr = 0;

    if(ioctl(share_memory, SHARE_MEMORY_ALIGN_ALLOC, &allocAlignMemOdOutput) < 0) 
    {
        std::cerr << "alloc allocAlignMemOdOutput error" << std::endl;
        std::abort();
    }

    virtual_addr_output = (char *)mmap(NULL, allocAlignMemOdOutput.size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_map, allocAlignMemOdOutput.phyAddr);
    if(virtual_addr_output == MAP_FAILED) 
    {
        std::cerr << "map allocAlignMemOdOutput error" << std::endl;
        std::abort();
    }
    
    virtualAddrOdOutput[0] = virtual_addr_output;
    output_pa_addr[0] = allocAlignMemOdOutput.phyAddr;
    for(uint32_t i = 1; i < 3; i++)
    {
        virtualAddrOdOutput[i] = virtualAddrOdOutput[i - 1] + output_size[i - 1];
        output_pa_addr[i] = output_pa_addr[i - 1] + output_size[i - 1];
    }

    for(uint32_t i = 0; i < GNNE_BUFFERS_COUNT; i++) 
    {
        allocAlignMemOdInput[i].size = input_size;
        allocAlignMemOdInput[i].alignment = MEMORY_TEST_BLOCK_ALIGN;
        allocAlignMemOdInput[i].phyAddr = 0;

        if(ioctl(share_memory, SHARE_MEMORY_ALIGN_ALLOC, &allocAlignMemOdInput[i]) < 0) 
        {
            std::cerr << "alloc allocAlignMemOdInput error" << std::endl;
            std::abort();
        }
        virtual_addr_input[i] = (char *)mmap(NULL, allocAlignMemOdInput[i].size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_map, allocAlignMemOdInput[i].phyAddr);
        if(virtual_addr_input[i] == MAP_FAILED) 
        {
            std::cerr << "map allocAlignMemOdInput error" << std::endl;
            std::abort();
        }
    }
}

void objectDetect::prepare_memory_output()
{
    output_size[0] = (FEATURE_LEN * (net_len / 8) * (net_len / 8) * sizeof(float));
    output_size[1] = (FEATURE_LEN * (net_len / 16) * (net_len / 16) * sizeof(float));
    output_size[2] = (FEATURE_LEN * (net_len / 32) * (net_len / 32) * sizeof(float));
    output_all_size = ((output_size[0] + output_size[1] + output_size[2] + 4095) & (~4095));

    allocAlignMemOdOutput.size = output_all_size;
    allocAlignMemOdOutput.alignment = MEMORY_TEST_BLOCK_ALIGN;
    allocAlignMemOdOutput.phyAddr = 0;

    if (ioctl(share_memory, SHARE_MEMORY_ALIGN_ALLOC, &allocAlignMemOdOutput) < 0)
    {
        std::cerr << "alloc allocAlignMemOdOutput error" << std::endl;
        std::abort();
    }

    virtual_addr_output = (char*)mmap(NULL, allocAlignMemOdOutput.size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_map, allocAlignMemOdOutput.phyAddr);
    if (virtual_addr_output == MAP_FAILED)
    {
        std::cerr << "map allocAlignMemOdOutput error" << std::endl;
        std::abort();
    }

    virtualAddrOdOutput[0] = virtual_addr_output;
    output_pa_addr[0] = allocAlignMemOdOutput.phyAddr;
    for (uint32_t i = 1; i < 3; i++)
    {
        virtualAddrOdOutput[i] = virtualAddrOdOutput[i - 1] + output_size[i - 1];
        output_pa_addr[i] = output_pa_addr[i - 1] + output_size[i - 1];
    }
}

void objectDetect::set_input(uint32_t index,  cv::Mat& static_img)
{
    auto in_shape = interp_od.input_shape(0);

    //cv::Mat static_img = cv::imread("./000001.jpg");
    std::vector<uchar> out_img;
    pre_post(static_img, out_img);
    memcpy(virtual_addr_input[index], out_img.data(), net_len * net_len * INPUT_CHANNELS);

    auto input_tensor = host_runtime_tensor::create(dt_uint8, in_shape,
        { (gsl::byte*)virtual_addr_input[index], net_len * net_len * INPUT_CHANNELS },
        false, hrt::pool_shared, allocAlignMemOdInput[index].phyAddr)
        .expect("cannot create input tensor");
    interp_od.input_tensor(0, input_tensor).expect("cannot set input tensor");
}

void objectDetect::set_input(uint32_t index)
{
    auto in_shape = interp_od.input_shape(0);
    auto input_tensor = host_runtime_tensor::create(dt_uint8, in_shape,
        { (gsl::byte *)virtual_addr_input[index], net_len * net_len * INPUT_CHANNELS},
        false, hrt::pool_shared, allocAlignMemOdInput[index].phyAddr)
                            .expect("cannot create input tensor");
    interp_od.input_tensor(0, input_tensor).expect("cannot set input tensor");

    // string file_name="./out_bin/input_"+std::to_string(r1_count)+".bin";
    // std::ofstream outF(file_name, std::ios::binary);
    // outF.write(reinterpret_cast<char*>(virtual_addr_input[index]),
    //         net_len * net_len * INPUT_CHANNELS*1);

    // outF.close();
}

void objectDetect::set_output()
{
    for (size_t i = 0; i < interp_od.outputs_size(); i++)
    {
        auto out_shape = interp_od.output_shape(i);
        auto output_tensor = host_runtime_tensor::create(dt_float32, out_shape,
        { (gsl::byte *)virtualAddrOdOutput[i], output_size[i]},
        false, hrt::pool_shared, output_pa_addr[i])
                            .expect("cannot create output tensor");

        interp_od.output_tensor(i, output_tensor).expect("cannot set output tensor");
    }
}

void objectDetect::load_model(char *path)
{
    od_model = read_binary_file<unsigned char>(path);
    interp_od.load_model({ (const gsl::byte *)od_model.data(), od_model.size() }).expect("cannot load model.");
    std::cout << "============> interp_od.load_model finished!" << std::endl;
}

void objectDetect::pre_post(cv::Mat& raw_img,vector<uchar>& out_img)
{
    ////std::vector<uchar> out_img(img_size * channels);
    //std::cout << "after pre_post" << std::endl;
    ////未对原图进行修改
    //cv::Mat tmp_img;
    ////BGR2RGB
    //cv::cvtColor(raw_img, tmp_img, cv::COLOR_BGR2RGB);
    ////padding
    //float width_ratio = float(raw_img.cols) / net_len;
    //float height_ratio = float(raw_img.rows)/ net_len;
    //float ratio = (width_ratio>=height_ratio)?width_ratio:height_ratio;
    //
    //uint32_t equal_trans_width = raw_img.cols / ratio;
    //uint32_t equal_trans_height = raw_img.rows / ratio;
   
    //uint32_t left, top, right, bottom;
    //if (equal_trans_width >= equal_trans_height)
    //{
    //    //上下padding PADDING_B
    //    top = abs(int(equal_trans_height - net_len)) / 2 * ratio;
    //    bottom = top;
    //    left = 0;
    //    right = 0;
    //}
    //else
    //{
    //    //左右padding
    //    top = 0;
    //    bottom = 0;
    //    left = abs(int(equal_trans_width - net_len)) / 2 * ratio;
    //    right = left;
    //}
    //std::cout << "top,bottom,let,right" << top <<" "<<bottom<<" "<<left<<" "<<right << std::endl;
    //std::cout << "before copyMakeBorder" << std::endl;
    //cv::copyMakeBorder(tmp_img, tmp_img, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(PADDING_R, PADDING_G, PADDING_B));
    //std::cout << "before resize" << std::endl;
    ////resize,cv::Size(w,h)
    //cv::resize(tmp_img, tmp_img, cv::Size(net_len, net_len), 0, 0, cv::INTER_LINEAR);

    //std::cout << "before HWC2CHW" << std::endl;
    ////HWC2CHW
    //std::vector<cv::Mat> rgb;
    //cv::split(tmp_img, rgb);

    ////std::array<uchar, img_size*channels> dst;    //此时应该使用可变长度的vector,而不是固定长度的std::array
    ////transposed HWC to CHW
    ////申请空间
    //size_t img_size = tmp_img.rows * tmp_img.cols;
    //if (out_img.size() != tmp_img.channels() * img_size)
    //{
    //    out_img.clear();
    //    out_img.resize(tmp_img.channels() * img_size);
    //}
    ////将3个通道依次放入到dst中，及C*H*W
    //for (int i = 0; i < tmp_img.channels(); ++i)
    //{
    //    //3个通道
    //    memcpy(out_img.data() + i * img_size, rgb[i].data, sizeof(uchar) * img_size);
    //}
    //std::cout << "after HWC2CHW" << std::endl;
    //padding
    //未对原图进行修改
    cv::Mat tmp_img;

    //padding
    float width_ratio = float(raw_img.cols) / net_len;
    float height_ratio = float(raw_img.rows) / net_len;
    float ratio = (width_ratio >= height_ratio) ? width_ratio : height_ratio;

    uint32_t equal_trans_width = raw_img.cols / ratio;
    uint32_t equal_trans_height = raw_img.rows / ratio;

    uint32_t left, top, right, bottom;
    uint32_t new_width, new_height;
    if (equal_trans_width >= equal_trans_height)
    {
        //上下padding PADDING_B
        top = abs(int(equal_trans_height - net_len)) / 2;
        bottom = top;
        left = 0;
        right = 0;
    }
    else
    {
        //左右padding
        top = 0;
        bottom = 0;
        left = abs(int(equal_trans_width - net_len)) / 2;
        right = left;
    }
    //resize,cv::Size(w,h)
   //cv::resize(tmp_img, tmp_img, cv::Size(net_len, net_len), 0, 0, cv::INTER_LINEAR);
    std::cout << "before resize" << std::endl;
    cv::resize(raw_img, tmp_img, cv::Size(equal_trans_width, equal_trans_height), 0, 0, cv::INTER_AREA);
    std::cout << "top,bo" << std::endl;
    std::cout << "before copyMakeBorder" << std::endl;
    cv::copyMakeBorder(tmp_img, tmp_img, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

    std::cout << "before HWC2CHW" << std::endl;
    //BGR2RGB
    cv::cvtColor(tmp_img, tmp_img, cv::COLOR_BGR2RGB);
    //HWC2CHW
    std::vector<cv::Mat> rgb;
    cv::split(tmp_img, rgb);

    //std::array<uchar, img_size*channels> dst;    //此时应该使用可变长度的vector,而不是固定长度的std::array
    //transposed HWC to CHW
    //申请空间
    size_t img_size = tmp_img.rows * tmp_img.cols;
    if (out_img.size() != tmp_img.channels() * img_size)
    {
        out_img.clear();
        out_img.resize(tmp_img.channels() * img_size);
    }
    //将3个通道依次放入到dst中，及C*H*W
    for (int i = 0; i < tmp_img.channels(); ++i)
    {
        //3个通道
        memcpy(out_img.data() + i * img_size, rgb[i].data, sizeof(uchar) * img_size);
    }

}


//void objectDetect::pre_process(cv::Mat& raw_img, vector<uchar>& out_img)
//{
//    //未对原图进行修改
//    cv::Mat tmp_img;
//
//    //padding
//    float width_ratio = float(raw_img.cols) / net_len;
//    float height_ratio = float(raw_img.rows) / net_len;
//    float ratio = (width_ratio >= height_ratio) ? width_ratio : height_ratio;
//
//    uint32_t equal_trans_width = raw_img.cols / ratio;
//    uint32_t equal_trans_height = raw_img.rows / ratio;
//
//    uint32_t left, top, right, bottom;
//    uint32_t new_width, new_height;
//    if (equal_trans_width >= equal_trans_height)
//    {
//        //上下padding PADDING_B
//        top = abs(int(equal_trans_height - net_len)) / 2;
//        bottom = top;
//        left = 0;
//        right = 0;
//    }
//    else
//    {
//        //左右padding
//        top = 0;
//        bottom = 0;
//        left = abs(int(equal_trans_width - net_len)) / 2;
//        right = left;
//    }
//    //resize,cv::Size(w,h)
//   //cv::resize(tmp_img, tmp_img, cv::Size(net_len, net_len), 0, 0, cv::INTER_LINEAR);
//    std::cout << "before resize" << std::endl;
//    cv::resize(raw_img, tmp_img, cv::Size(equal_trans_width, equal_trans_height), 0, 0, cv::INTER_AREA);
//    std::cout << "top,bo" << std::endl;
//    std::cout << "before copyMakeBorder" << std::endl;
//    cv::copyMakeBorder(tmp_img, tmp_img, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
//
//    std::cout << "before HWC2CHW" << std::endl;
//    //BGR2RGB
//    cv::cvtColor(tmp_img, tmp_img, cv::COLOR_BGR2RGB);
//    //HWC2CHW
//    std::vector<cv::Mat> rgb;
//    cv::split(tmp_img, rgb);
//
//    //std::array<uchar, img_size*channels> dst;    //此时应该使用可变长度的vector,而不是固定长度的std::array
//    //transposed HWC to CHW
//    //申请空间
//    size_t img_size = tmp_img.rows * tmp_img.cols;
//    if (out_img.size() != tmp_img.channels() * img_size)
//    {
//        out_img.clear();
//        out_img.resize(tmp_img.channels() * img_size);
//    }
//    //将3个通道依次放入到dst中，及C*H*W
//    for (int i = 0; i < tmp_img.channels(); ++i)
//    {
//        //3个通道
//        memcpy(out_img.data() + i * img_size, rgb[i].data, sizeof(uchar) * img_size);
//    }
//
//}
void objectDetect::run()
{
    interp_od.run().expect("error occurred in running model");
}

void objectDetect::run(cv::Mat& img)
{
    //可能每张图像的大小不同
    frame_size = { img.cols,img.rows};    //返回原图对应的大小或者等比例缩放后的大小
    std::cout << frame_size.width<<" "<<frame_size.height << std::endl;
    std::vector<uchar> out_img;
    pre_post(img, out_img);
    cv::Mat image_static = cv::imread("./test.jpg");
    auto in_shape = interp_od.input_shape(0);
    auto input_tensor = host_runtime_tensor::create(dt_uint8, in_shape,
        { (gsl::byte*)(image_static.data), net_len * net_len * img.channels() },
        false, hrt::pool_shared).expect("cannot create input tensor");

    /*auto input_tensor = host_runtime_tensor::create(dt_uint8, in_shape,
        { (gsl::byte*)virtual_addr_input[index], net_len * net_len * INPUT_CHANNELS },
        false, hrt::pool_shared, allocAlignMemOdInput[index].phyAddr)
        .expect("cannot create input tensor");*/
    interp_od.input_tensor(0, input_tensor).expect("cannot set input tensor");
    interp_od.run().expect("error occurred in running model");
    std::cout << "after run" << std::endl;
}

void objectDetect::get_output()
{
    output_0 = reinterpret_cast<float *>(virtualAddrOdOutput[0]);
    output_1 = reinterpret_cast<float *>(virtualAddrOdOutput[1]);
    output_2 = reinterpret_cast<float *>(virtualAddrOdOutput[2]);
    // for(int i = 0;i < 3;i++)
    // {
    //     string file_name="./out_bin/output_"+std::to_string(r1_count++)+std::to_string(i)+".bin";
    //     std::ofstream outF(file_name, std::ios::binary);
    //     outF.write(reinterpret_cast<char*>(virtualAddrOdOutput[i]),output_size[i]);
    // }

}

// 后处理
void objectDetect::post_process(std::vector<BoxInfo>& result,DETECTION_ROWS& detections)
{
    auto boxes0 = decode_infer(output_0, net_len, 8, classes_num, frame_size, anchors_0, obj_thresh);
    result.insert(result.begin(), boxes0.begin(), boxes0.end());
    auto boxes1 = decode_infer(output_1, net_len, 16, classes_num, frame_size, anchors_1, obj_thresh);
    result.insert(result.begin(), boxes1.begin(), boxes1.end());
    auto boxes2 = decode_infer(output_2, net_len, 32, classes_num, frame_size, anchors_2, obj_thresh);
    result.insert(result.begin(), boxes2.begin(), boxes2.end());
    nms(result, nms_thresh, detections);
}

objectDetect::objectDetect(float obj_thresh, float nms_thresh, int net_len, Framesize frame_size)
:obj_thresh(obj_thresh), nms_thresh(nms_thresh), net_len(net_len), frame_size(frame_size)
{
    share_memory = open(SHARE_MEMORY_DEV, O_RDWR);
    if(share_memory < 0) 
    {
        std::cerr << "open /dev/k510-share-memory error" << std::endl;
        std::abort();
    }
    mem_map = open(MAP_MEMORY_DEV, O_RDWR | O_SYNC);
    if (mem_map < 0) 
    {
        std::cerr << "open /dev/mem error" << std::endl;
        std::abort();
    }
}


objectDetect::~objectDetect()
{
    for(uint32_t i = 0; i < GNNE_BUFFERS_COUNT; i++) 
    {
        if(virtual_addr_input[i])
            munmap(virtual_addr_input[i], allocAlignMemOdInput[i].size);

        if(allocAlignMemOdInput[i].phyAddr != 0)
        {
            if(ioctl(share_memory, SHARE_MEMORY_FREE, &allocAlignMemOdInput[i].phyAddr) < 0) 
            {
                std::cerr << "free allocAlignMemOdInput error" << std::endl;
                std::abort();
            }
        }
    }
    if(virtual_addr_output)
        munmap(virtual_addr_output, allocAlignMemOdOutput.size);

    if(allocAlignMemOdOutput.phyAddr != 0)
    {
        if(ioctl(share_memory, SHARE_MEMORY_FREE, &allocAlignMemOdOutput.phyAddr) < 0) 
        {
            std::cerr << "free allocAlignMemOdOutput error" << std::endl;
            std::abort();
        }
    }
    close(share_memory);
    close(mem_map);
}
