#ifndef TRACKER_H
#define TRACKER_H
#include <vector>

#include "kalmanfilter.h"
#include "track.h"
#include "definition/config.h"

class NearNeighborDisMetric;

class tracker
{
public:
    float max_iou_distance;
    int max_age;
    int n_init;
    int _next_idx;
    int max_hits;

    NearNeighborDisMetric *metric;
    DeepSortKalmanFilter *kf;

public:
    std::vector<Track> tracks;
    tracker(/*NearNeighborDisMetric* metric,*/
            float iou_distance = MAX_IOU_DISTANCE, int n_init = MAX_N_INIT,int max_hits = MAX_HITS,
            float max_cosine_distance=MAX_COSINE_DISTANCE, int nn_budget=MAX_NN_BUDGET,
            int age = MAX_AGE);
        
    ~tracker();
    void predict();
    void update(const DETECTION_ROWS& detections, DEEP_SORT_RET& ret);
    void update(const DETECTION_REID_ROWS &detections);
#ifdef MAKEMD
    void showTracks();
#endif
    typedef DYNAMICM (tracker::*GATED_METRIC_FUNC_NO_FEAT)(
        std::vector<Track> &tracks,
        const DETECTION_ROWS &dets,
        const std::vector<int> &track_indices,
        const std::vector<int> &detection_indices);
    typedef DYNAMICM(tracker::* GATED_METRIC_FUNC)(
        std::vector<Track>& tracks,
        const DETECTION_REID_ROWS& dets,
        const std::vector<int>& track_indices,
        const std::vector<int>& detection_indices);

private:
    void _match(const DETECTION_ROWS& detections, TRACHER_MATCHD& res);
    void _match(const DETECTION_REID_ROWS &detections, TRACHER_MATCHD &res);
    void _initiate_track(const DETECTION_ROW& detection);
    void _initiate_track(const DETECTION_REID_ROW &detection);

public:
    DYNAMICM gated_matric(
        std::vector<Track>& tracks,
        const DETECTION_REID_ROWS& dets,
        const std::vector<int>& track_indices,
        const std::vector<int>& detection_indices);

    DYNAMICM iou_cost_no_feat(
        std::vector<Track>& tracks,
        const DETECTION_ROWS& dets,
        const std::vector<int>& track_indices,
        const std::vector<int>& detection_indices);
    DYNAMICM iou_cost(
        std::vector<Track> &tracks,
        const DETECTION_REID_ROWS &dets,
        const std::vector<int> &track_indices,
        const std::vector<int> &detection_indices);
    Eigen::VectorXf iou(DETECTBOX &bbox,
                        DETECTBOXSS &candidates);
};

#endif // TRACKER_H
