#ifndef DATATYPE_H
#define DATATYPE_H

#include <cstddef>
#include <vector>
#include<string>
#include <Eigen/Core>
#include"config.h"

using std::vector;
using std::string;

typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> DETECTBOX;
typedef Eigen::Matrix<float, -1, 4, Eigen::RowMajor> DETECTBOXSS;

typedef Eigen::Matrix<float, 1, FEATURE_LEN, Eigen::RowMajor> FEATURE;
typedef Eigen::Matrix<float, Eigen::Dynamic, FEATURE_LEN, Eigen::RowMajor> FEATURESS;
//typedef std::vector<FEATURE> FEATURESS;

//DeepSortKalmanFilteranFilter
//typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_FILTER;
typedef Eigen::Matrix<float, 1, 8, Eigen::RowMajor> KAL_MEAN;
typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_COVA;
typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> KAL_HMEAN;
typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> KAL_HCOVA;
using KAL_DATA = std::pair<KAL_MEAN, KAL_COVA>;
using KAL_HDATA = std::pair<KAL_HMEAN, KAL_HCOVA>;

//main
using RESULT_DATA = std::pair<int, DETECTBOX>;

//tracker:
using TRACKER_DATA = std::pair<int, FEATURESS>;
using MATCH_DATA = std::pair<int, int>;
typedef struct t
{
    vector<MATCH_DATA> matches;
    vector<int> unmatched_tracks;
    vector<int> unmatched_detections;
} TRACHER_MATCHD;

//linear_assignment:
typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor> DYNAMICM;

//for detection model
enum DETECTBOX_IDX
{
    IDX_X = 0,
    IDX_Y,
    IDX_W,
    IDX_H
};

class DETECTION_ROW
{
public:
    DETECTBOX tlwh;   //np.float
    float confidence; //float
    DETECTBOX to_xyah() const;
    DETECTBOX to_tlbr() const;
};
typedef vector<DETECTION_ROW> DETECTION_ROWS;

//for detection & reid model
class DETECTION_REID_ROW:public DETECTION_ROW
{
public:
    FEATURE feature;  //np.float32
};
typedef vector<DETECTION_REID_ROW> DETECTION_REID_ROWS;

// enum ImageExt
// {
//     JPG,
//     JPEG,
//     PNG
// };
const string img_exts[3]={".jpg",".jpeg",".png"};

typedef struct DeepSortTrackRet
{
   int track_id;
   DETECTBOX track_bbox;

   DeepSortTrackRet(int &track_id, DETECTBOX track_bbox)
   {
       this->track_id = track_id;
       this->track_bbox = track_bbox;
   }

}DEEP_SORT_TRACK_RET;

typedef struct
{
    vector<int> disp_track_id;
    vector<DEEP_SORT_TRACK_RET> tracks_ret;
} DEEP_SORT_RET;
#endif // DATATYPE_H

