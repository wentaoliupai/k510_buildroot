#include "cv2_utils.h"

void nms(std::vector<BoxInfo>& input_boxes, float nms_thresh, DETECTION_ROWS& output_boxes)
{
    std::sort(input_boxes.begin(), input_boxes.end(), [](BoxInfo a, BoxInfo b) { return a.score > b.score; });
    std::vector<float> vArea(input_boxes.size());
    for (int i = 0; i < int(input_boxes.size()); ++i)
    {
        vArea[i] = (input_boxes.at(i).x2 - input_boxes.at(i).x1 + 1)
            * (input_boxes.at(i).y2 - input_boxes.at(i).y1 + 1);
    }
    for (int i = 0; i < int(input_boxes.size()); ++i)
    {
        //因为xywh的存储，可能需要一次额外的内存拷贝，后期可以优化
        DETECTION_ROW out;
        out.tlwh[IDX_X] = input_boxes.at(i).x1;
        out.tlwh[IDX_Y] = input_boxes.at(i).y1;
        out.tlwh[IDX_W] = input_boxes.at(i).x2 - input_boxes.at(i).x1 + 1;
        out.tlwh[IDX_H] = input_boxes.at(i).y2 - input_boxes.at(i).y1 + 1;
        out.confidence = input_boxes.at(i).score;
        output_boxes.push_back(out);
        for (int j = i + 1; j < int(input_boxes.size());)
        {
            float xx1 = std::max(input_boxes[i].x1, input_boxes[j].x1);
            float yy1 = std::max(input_boxes[i].y1, input_boxes[j].y1);
            float xx2 = std::min(input_boxes[i].x2, input_boxes[j].x2);
            float yy2 = std::min(input_boxes[i].y2, input_boxes[j].y2);
            float w = std::max(float(0), xx2 - xx1 + 1);
            float h = std::max(float(0), yy2 - yy1 + 1);
            float inter = w * h;
            float ovr = inter / (vArea[i] + vArea[j] - inter);
            if (ovr >= nms_thresh)
            {
                input_boxes.erase(input_boxes.begin() + j);
                vArea.erase(vArea.begin() + j);
            }
            else
            {
                j++;
            }
        }
        
       
    }
    /*for (auto iter = input_boxes.begin(); iter != input_boxes.end(); ++iter)
    {
        std::cout<<"nms x1,x2,y1,y2" << iter->x1 << " " << iter->x2 << " " << iter->y1 << " " << iter->y2 << std::endl;
    }
    std::cout << "input_boxes::len" << input_boxes.size() << std::endl;*/
}

std::vector<BoxInfo> decode_infer(float* data, int net_size, int stride, int num_classes, Framesize frame_size, float anchors[][2], float threshold)
{
    float ratiow = (float)net_size / frame_size.width;
    float ratioh = (float)net_size / frame_size.height;
    float gain = ratiow < ratioh ? ratiow : ratioh;
    std::vector<BoxInfo> result;
    int grid_size = net_size / stride;
    int one_rsize = num_classes + 5;
    float cx, cy, w, h;
    for (int shift_y = 0; shift_y < grid_size; shift_y++)
    {
        for (int shift_x = 0; shift_x < grid_size; shift_x++)
        {
            int loc = shift_x + shift_y * grid_size;
            for (int i = 0; i < 3; i++)
            {
                float* record = data + (loc * 3 + i) * one_rsize;
                float* cls_ptr = record + 5;
                //std::cout << "threshold:" << threshold << std::endl;
                float score = 0,max_score=0;
                int cls_label=0;
                for (int cls = 0; cls < 1; cls++)
                {
                    score = (cls_ptr[cls]) * (record[4]);
                    if (score > max_score)
                    {
                        max_score = score;
                        cls_label = cls;
                    }
                }
                if (max_score > threshold)
                {
                    cx = ((record[0]) * 2.f - 0.5f + (float)shift_x) * (float)stride;
                    cy = ((record[1]) * 2.f - 0.5f + (float)shift_y) * (float)stride;
                    w = pow((record[2]) * 2.f, 2) * anchors[i][0];
                    h = pow((record[3]) * 2.f, 2) * anchors[i][1];
                    cx -= ((net_size - frame_size.width * gain) / 2);
                    cy -= ((net_size - frame_size.height * gain) / 2);
                    cx /= gain;
                    cy /= gain;
                    w /= gain;
                    h /= gain;
                    BoxInfo box;
                    box.x1 = std::max(0, std::min(frame_size.width, int(cx - w / 2.f)));
                    box.y1 = std::max(0, std::min(frame_size.height, int(cy - h / 2.f)));
                    box.x2 = std::max(0, std::min(frame_size.width, int(cx + w / 2.f)));
                    box.y2 = std::max(0, std::min(frame_size.height, int(cy + h / 2.f)));
                    box.score = max_score;
                    box.label = cls_label;
                    result.push_back(box);
                }
            }
        }
    }
    //std::cout << "result.len" << result.size() << std::endl;
    return result;
}