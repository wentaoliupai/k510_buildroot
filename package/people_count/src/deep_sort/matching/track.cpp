#include "track.h"
#include "definition/config.h"
#include "definition/data_type.h"
////#include"utils/util_logger.h"
#include <iostream>

Track::Track(KAL_MEAN& mean, KAL_COVA& covariance, int track_id, int n_init,int max_hits)
{
    this->mean = mean;
    this->covariance = covariance;
    this->track_id = track_id;
    this->hits = 1;
    this->age = 1;
    this->time_since_update = 0;
    this->state = TrackState::Tentative;
    this->_n_init = n_init;
    this->_max_age = 0;             //只要有一帧丢掉，就丢掉
    this->max_hits = max_hits;
    //std::cout<<"Track::Track:this->max_hits"<<this->max_hits<<std::endl;
    //LOG_WITH_DEBUG_LEVEL("New Track:{}", track_id);
}

Track::Track(KAL_MEAN &mean, KAL_COVA &covariance, int track_id, int n_init, int max_age, const FEATURE &feature)
{
    this->mean = mean;
    this->covariance = covariance;
    this->track_id = track_id;
    this->hits = 1;
    this->age = 1;
    this->time_since_update = 0;
    this->state = TrackState::Tentative;
    features = FEATURESS(1, FEATURE_LEN);
    features.row(0) = feature; //features.rows() must = 0;

    this->_n_init = n_init;
    this->_max_age = max_age;
    //LOG_WITH_DEBUG_LEVEL("New Track:{}",track_id);
}

Track::~Track()
{
    //LOG_WITH_DEBUG_LEVEL("~ Track");
}

void Track::predit(DeepSortKalmanFilter *kf)
{
    kf->predict(this->mean, this->covariance);
    this->age += 1;
    this->time_since_update += 1;
}

void Track::update(DeepSortKalmanFilter* const kf, const DETECTION_ROW& detection,DEEP_SORT_RET& ret)
{
    det_bbox = detection.tlwh;
    KAL_DATA pa = kf->update(this->mean, this->covariance, detection.to_xyah());
    this->mean = pa.first;
    this->covariance = pa.second;
    //    this->features.row(features.rows()) = detection.feature;
    this->hits += 1;
    //std::cout<<"this-hits:,max_hits:"<<this->hits<<" "<<max_hits<<std::endl;
    this->time_since_update = 0;
    if (this->state == TrackState::Tentative && this->hits >= this->_n_init)
    {
        this->state = TrackState::Confirmed;
    }
    if((this->hits-this->_n_init) >= max_hits)
    {
        this->state = TrackState::Deleted;
        //若是只有confirmed才记录删除，那么会删除该轨迹时，会少前n帧的特征
        ret.disp_track_id.push_back(this->track_id); 
    }
}

void Track::update(DeepSortKalmanFilter *const kf, const DETECTION_REID_ROW &detection)
{
    KAL_DATA pa = kf->update(this->mean, this->covariance, detection.to_xyah());
    this->mean = pa.first;
    this->covariance = pa.second;

    featuresAppendOne(detection.feature);
    //    this->features.row(features.rows()) = detection.feature;
    this->hits += 1;
    
    this->time_since_update = 0;
    if (this->state == TrackState::Tentative && this->hits >= this->_n_init)
    {
        this->state = TrackState::Confirmed;
    }
}

void Track::mark_missed()
{
    if (this->state == TrackState::Tentative)
    {
        this->state = TrackState::Deleted;
    }
    else if (this->time_since_update > this->_max_age)
    {
        this->state = TrackState::Deleted;
    }
}

void Track::mark_missed(DEEP_SORT_RET& ret)
{
    if (this->state == TrackState::Tentative)
    {
        this->state = TrackState::Deleted;
    }
    else if (this->time_since_update > this->_max_age)
    {
        this->state = TrackState::Deleted;
        //若是只有confirmed才记录删除，那么会删除该轨迹时，会少前n帧的特征
        ret.disp_track_id.push_back(this->track_id);        
    }
    
}

bool Track::is_confirmed()
{
    return this->state == TrackState::Confirmed;
}

bool Track::is_deleted()
{
    return this->state == TrackState::Deleted;
}

bool Track::is_tentative()
{
    return this->state == TrackState::Tentative;
}

DETECTBOX Track::to_tlwh()
{
    DETECTBOX ret = mean.leftCols(4);
    ret(2) *= ret(3);
    ret.leftCols(2) -= (ret.rightCols(2) / 2);
    return ret;
}

void Track::featuresAppendOne(const FEATURE &f)
{
    int size = this->features.rows();
    FEATURESS newfeatures = FEATURESS(size + 1, FEATURE_LEN);
    newfeatures.block(0, 0, size, FEATURE_LEN) = this->features;    //原featrue
    newfeatures.row(size) = f;                                                      //新featrue
    features = newfeatures;
}
