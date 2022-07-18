#ifndef LINEAR_ASSIGNMENT_H
#define LINEAR_ASSIGNMENT_H
#include "definition/data_type.h"
#include "tracker.h"

#define INFTY_COST 1e5
class tracker;
//for matching;
class linear_assignment
{
public:
    static TRACHER_MATCHD matching_cascade(tracker *distance_metric,
                                    tracker::GATED_METRIC_FUNC distance_metric_func,
                                    float max_distance,
                                    int cascade_depth,
                                    std::vector<Track> &tracks,
                                    const DETECTION_REID_ROWS &detections,
                                    std::vector<int> &track_indices,
                                    std::vector<int> detection_indices = std::vector<int>());
    
    static TRACHER_MATCHD min_cost_matching(
        tracker* distance_metric,
        tracker::GATED_METRIC_FUNC_NO_FEAT distance_metric_func,
        float max_distance,
        std::vector<Track>& tracks,
        const DETECTION_ROWS& detections,
        std::vector<int>& track_indices,
        std::vector<int>& detection_indices);
    static TRACHER_MATCHD min_cost_matching(
        tracker *distance_metric,
        tracker::GATED_METRIC_FUNC distance_metric_func,
        float max_distance,
        std::vector<Track> &tracks,
        const DETECTION_REID_ROWS &detections,
        std::vector<int> &track_indices,
        std::vector<int> &detection_indices);

    static DYNAMICM gate_cost_matrix(
        DeepSortKalmanFilter *kf,
        DYNAMICM &cost_matrix,
        std::vector<Track> &tracks,
        const DETECTION_REID_ROWS &detections,
        const std::vector<int> &track_indices,
        const std::vector<int> &detection_indices,
        float gated_cost = INFTY_COST,
        bool only_position = false);
};

#endif // LINEAR_ASSIGNMENT_H
