#include "tracker.h"
#include "nn_matching.h"
#include "linear_assignment.h"
//#include"utils/util_logger.h"

#ifdef MAKEMD
#include <iostream>
#endif
#include <iostream>

tracker::tracker(/*NearNeighborDisMetric* metric,*/
            float iou_distance, int n_init,int max_hits,
            float max_cosine_distance, int nn_budget,
            int age)
{
    this->metric = new NearNeighborDisMetric(
        NearNeighborDisMetric::METRIC_TYPE::cosine,
        max_cosine_distance, nn_budget);
    this->max_iou_distance = iou_distance;
    this->max_age = age;
    this->n_init = n_init;

    this->kf = new DeepSortKalmanFilter();
    this->tracks.clear();
    this->_next_idx = 1;
    this->max_hits = max_hits;
    //std::cout<<"tracker::tracker:max_hits"<<max_hits<<std::endl;
}
tracker::~tracker()
{
    if (this->metric != nullptr)
        delete this->metric;
    if (this->kf != nullptr)
        delete this->kf;
    ////LOG_WITH_DEBUG_LEVEL("~ tracker");
}
void tracker::predict()
{
    for (Track &track:tracks)
    {
        track.predit(kf);
    }
}

void tracker::update(const DETECTION_ROWS& detections, DEEP_SORT_RET& ret)
{
    TRACHER_MATCHD res;
    //std::cout << "before _match" << std::endl;
    //1.match
    _match(detections, res);

    // std::cout << "before for matches" << std::endl;
    //2.for matches(update tracks)
    vector<MATCH_DATA>& matches = res.matches;
    for (MATCH_DATA& data : matches)
    {
        int track_idx = data.first;
        int detection_idx = data.second;
        tracks[track_idx].update(this->kf, detections[detection_idx],ret);
    }

    // std::cout << "before for unmatched_tracks" << std::endl;
    //3.for  unmatched_tracks(delete tracks)
    vector<int>& unmatched_tracks = res.unmatched_tracks;
    for (int& track_idx : unmatched_tracks)
    {
        this->tracks[track_idx].mark_missed(ret);
    }

    //std::cout << "before for ret" << std::endl;
    vector<Track>::iterator it;
    for (it = tracks.begin(); it != tracks.end();)
    {
        if ((*it).is_deleted())
        {
             it = tracks.erase(it);
        }
        else
        {
            if (it->is_confirmed())
            {
                auto a = it->to_tlwh();
                ret.tracks_ret.push_back(DeepSortTrackRet(it->track_id, it->to_tlwh()));
                //std::cout<<"track_id:"<<it->track_id<<"hits:"<<it->hits<<std::endl;
            }
            ++it;
        }
    }

    // std::cout << "before _initiate_track" << std::endl;
    //4.for unmatched_detections(new tracks)
    vector<int>& unmatched_detections = res.unmatched_detections;
    for (int& detection_idx : unmatched_detections)
    {
        this->_initiate_track(detections[detection_idx]);
    }
}

void tracker::update(const DETECTION_REID_ROWS &detections)
{
    TRACHER_MATCHD res;
    //1.match
    _match(detections, res);

    //2.for matches(update tracks)
    vector<MATCH_DATA> &matches = res.matches;
    for (MATCH_DATA &data : matches)
    {
        int track_idx = data.first;
        int detection_idx = data.second;
        tracks[track_idx].update(this->kf, detections[detection_idx]);
    }

    //3.for  unmatched_tracks(delete tracks)
    vector<int> &unmatched_tracks = res.unmatched_tracks;
    for (int &track_idx : unmatched_tracks)
    {
        this->tracks[track_idx].mark_missed();
    }
    vector<Track>::iterator it;
    for (it = tracks.begin(); it != tracks.end();)
    {
        if ((*it).is_deleted())
            it = tracks.erase(it);
        else
            ++it;
    }

    //4.for unmatched_detections(new tracks)
    vector<int> &unmatched_detections = res.unmatched_detections;
    for (int &detection_idx : unmatched_detections)
    {
        this->_initiate_track(detections[detection_idx]);
    }

    //5.保存det reid特征（for confirmed tracks）
    vector<int> active_targets;
    vector<TRACKER_DATA> tid_features;
    for (Track &track : tracks)
    {
        if (track.is_confirmed() == false)
            continue;
        active_targets.push_back(track.track_id);
        tid_features.push_back(std::make_pair(track.track_id,
                                              track.features));
        FEATURESS t = FEATURESS(0, FEATURE_LEN);
        track.features = t;
    }
    this->metric->partial_fit(tid_features, active_targets);
}

#ifdef MAKEMD
void tracker::showTracks()
{
    for (Track &track : tracks)
    {
        string str_state;
        switch (track.state)
        {
        case 1:
            str_state = "Tentative";
            break;
        case 2:
            str_state = "Confirmed";
            break;
        case 3:
            str_state = "Deleted";
            break;
        default:
            break;
        }
        std::cout << "track_id:" << track.track_id << ",state:" << str_state << ",hits:" << track.hits << ",time_since_update:" << track.time_since_update << std::endl;
    }
    std::cout << std::endl;
}
#endif

void tracker::_match(const DETECTION_ROWS& detections, TRACHER_MATCHD& res)
{
    //std::cout << "before construct confirmed_tracks" << std::endl;
    //1.construct confirmed_tracks &  unconfirmed_tracks
    vector<int> iou_track_candidates(tracks.size());
    for (int i = 0; i < tracks.size(); ++i)
    {
        iou_track_candidates[i] = i;
    }
    vector<int> unmatched_detections(detections.size());
    for (int i = 0; i < detections.size(); ++i)
    {
        unmatched_detections[i] = i;
    }
    //std::cout << "before construct confirmed_tracks" << std::endl;
    //4.iou match
    TRACHER_MATCHD matchb =
        linear_assignment::min_cost_matching(
            this,
            &tracker::iou_cost_no_feat,
            this->max_iou_distance,
            this->tracks,
            detections,
            iou_track_candidates,
            unmatched_detections);

    //std::cout << "before get result" << std::endl;
    //5.get result:
    //matches=matchb
    res.matches.insert(res.matches.end(), matchb.matches.begin(), matchb.matches.end());
    res.unmatched_tracks.insert(res.unmatched_tracks.end(),
        matchb.unmatched_tracks.begin(), matchb.unmatched_tracks.end());
    //unmatched_detections=matchb
    res.unmatched_detections.assign(
        matchb.unmatched_detections.begin(), matchb.unmatched_detections.end());
}

void tracker::_match(const DETECTION_REID_ROWS &detections, TRACHER_MATCHD &res)
{
    //1.construct confirmed_tracks &  unconfirmed_tracks
    vector<int> confirmed_tracks;
    vector<int> unconfirmed_tracks;
    int idx = 0;
    for (Track &t : tracks)
    {
        if (t.is_confirmed())
            confirmed_tracks.push_back(idx);
        else
            unconfirmed_tracks.push_back(idx);
        idx++;
    }

    //2.cascade match
    TRACHER_MATCHD matcha =
        linear_assignment::matching_cascade(
            this,
            &tracker::gated_matric,
            this->metric->mating_threshold,
            this->max_age,
            this->tracks,
            detections,
            confirmed_tracks);

    //3.construct iou_track_candidates=
    //unconfirmebd_tracks & matcha.unmatched_tracks(time_since_update == 1)
    vector<int> iou_track_candidates;
    iou_track_candidates.assign(unconfirmed_tracks.begin(), unconfirmed_tracks.end());
    vector<int>::iterator it;
    for (it = matcha.unmatched_tracks.begin(); it != matcha.unmatched_tracks.end();)
    {
        int idx = *it;
        if (tracks[idx].time_since_update == 1)
        { //push into unconfirmed
            iou_track_candidates.push_back(idx);
            it = matcha.unmatched_tracks.erase(it);
            continue;
        }
        ++it;
    }

    //4.iou match
    TRACHER_MATCHD matchb = 
        linear_assignment::min_cost_matching(
        this, 
        &tracker::iou_cost,
        this->max_iou_distance,
        this->tracks,
        detections,
        iou_track_candidates,
        matcha.unmatched_detections);

    //5.get result:
    //matches=matcha+matchb
    res.matches.assign(matcha.matches.begin(), matcha.matches.end());
    res.matches.insert(res.matches.end(), matchb.matches.begin(), matchb.matches.end());
    //unmatched_tracks=matcha(time_since_update>1)+matchb
    res.unmatched_tracks.assign(
        matcha.unmatched_tracks.begin(), matcha.unmatched_tracks.end());
    res.unmatched_tracks.insert(res.unmatched_tracks.end(),
                                matchb.unmatched_tracks.begin(), matchb.unmatched_tracks.end());
    //unmatched_detections=matchb
    res.unmatched_detections.assign(
        matchb.unmatched_detections.begin(), matchb.unmatched_detections.end());
}

void tracker::_initiate_track(const DETECTION_ROW& detection)
{
    KAL_DATA data = kf->initiate(detection.to_xyah());
    KAL_MEAN mean = data.first;
    KAL_COVA covariance = data.second;

    this->tracks.push_back(Track(mean, covariance, this->_next_idx, this->n_init,this->max_hits));
    _next_idx += 1;
}

void tracker::_initiate_track(const DETECTION_REID_ROW &detection)
{
    KAL_DATA data = kf->initiate(detection.to_xyah());
    KAL_MEAN mean = data.first;
    KAL_COVA covariance = data.second;

    this->tracks.push_back(Track(mean, covariance, this->_next_idx, this->n_init,
                                 this->max_age, detection.feature));
    _next_idx += 1;
}

//马氏距离 for 识别
DYNAMICM tracker::gated_matric(
    std::vector<Track> &tracks,
    const DETECTION_REID_ROWS &dets,
    const std::vector<int> &track_indices,
    const std::vector<int> &detection_indices)
{
    FEATURESS features(detection_indices.size(), FEATURE_LEN);
    int pos = 0;
    for (int i : detection_indices)
    {
        features.row(pos++) = dets[i].feature;
    }
    vector<int> targets;
    for (int i : track_indices)
    {
        targets.push_back(tracks[i].track_id);
    }
    //马氏距离
    DYNAMICM cost_matrix = this->metric->distance(features, targets);
#ifdef MAKEMD
    std::cout << cost_matrix << std::endl;
#endif
    DYNAMICM res = linear_assignment::gate_cost_matrix(
        this->kf, cost_matrix, tracks, dets, track_indices,
        detection_indices);
    return res;
}

DYNAMICM
tracker::iou_cost_no_feat(
    std::vector<Track>& tracks,
    const DETECTION_ROWS& dets,
    const std::vector<int>& track_indices,
    const std::vector<int>& detection_indices)
{
    int rows = track_indices.size();
    int cols = detection_indices.size();
    DYNAMICM cost_matrix = Eigen::MatrixXf::Zero(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        int track_idx = track_indices[i];
        if (tracks[track_idx].time_since_update > 1)
        {
            cost_matrix.row(i) = Eigen::RowVectorXf::Constant(cols, INFTY_COST);
            continue;
        }
        DETECTBOX bbox = tracks[track_idx].to_tlwh();
        int csize = detection_indices.size();
        DETECTBOXSS candidates(csize, 4);
        for (int k = 0; k < csize; k++)
            candidates.row(k) = dets[detection_indices[k]].tlwh;
        Eigen::RowVectorXf rowV = (1. - iou(bbox, candidates).array()).matrix().transpose();
        cost_matrix.row(i) = rowV;
    }
    return cost_matrix;
}

DYNAMICM
tracker::iou_cost(
    std::vector<Track> &tracks,
    const DETECTION_REID_ROWS &dets,
    const std::vector<int> &track_indices,
    const std::vector<int> &detection_indices)
{
    //!!!python diff: track_indices && detection_indices will never be None.
    //    if(track_indices.empty() == true) {
    //        for(size_t i = 0; i < tracks.size(); i++) {
    //            track_indices.push_back(i);
    //        }
    //    }
    //    if(detection_indices.empty() == true) {
    //        for(size_t i = 0; i < dets.size(); i++) {
    //            detection_indices.push_back(i);
    //        }
    //    }
    int rows = track_indices.size();
    int cols = detection_indices.size();
    DYNAMICM cost_matrix = Eigen::MatrixXf::Zero(rows, cols);
    for (int i = 0; i < rows; i++)
    {
        int track_idx = track_indices[i];
        if (tracks[track_idx].time_since_update > 1)
        {
            cost_matrix.row(i) = Eigen::RowVectorXf::Constant(cols, INFTY_COST);
            continue;
        }
        DETECTBOX bbox = tracks[track_idx].to_tlwh();
        int csize = detection_indices.size();
        DETECTBOXSS candidates(csize, 4);
        for (int k = 0; k < csize; k++)
            candidates.row(k) = dets[detection_indices[k]].tlwh;
        Eigen::RowVectorXf rowV = (1. - iou(bbox, candidates).array()).matrix().transpose();
        cost_matrix.row(i) = rowV;
    }
    return cost_matrix;
}

Eigen::VectorXf
tracker::iou(DETECTBOX &bbox, DETECTBOXSS &candidates)
{
    float bbox_tl_1 = bbox[0];
    float bbox_tl_2 = bbox[1];
    float bbox_br_1 = bbox[0] + bbox[2];
    float bbox_br_2 = bbox[1] + bbox[3];
    float area_bbox = bbox[2] * bbox[3];

    Eigen::Matrix<float, -1, 2> candidates_tl;
    Eigen::Matrix<float, -1, 2> candidates_br;
    candidates_tl = candidates.leftCols(2);
    candidates_br = candidates.rightCols(2) + candidates_tl;

    int size = int(candidates.rows());
    //    Eigen::VectorXf area_intersection(size);
    //    Eigen::VectorXf area_candidates(size);
    Eigen::VectorXf res(size);
    for (int i = 0; i < size; i++)
    {
        float tl_1 = std::max(bbox_tl_1, candidates_tl(i, 0));
        float tl_2 = std::max(bbox_tl_2, candidates_tl(i, 1));
        float br_1 = std::min(bbox_br_1, candidates_br(i, 0));
        float br_2 = std::min(bbox_br_2, candidates_br(i, 1));

        float w = br_1 - tl_1;
        w = (w < 0 ? 0 : w);
        float h = br_2 - tl_2;
        h = (h < 0 ? 0 : h);
        float area_intersection = w * h;
        float area_candidates = candidates(i, 2) * candidates(i, 3);
        res[i] = area_intersection / (area_bbox + area_candidates - area_intersection);
    }
    return res;
}
