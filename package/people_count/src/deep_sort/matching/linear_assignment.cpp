#include<map>
#include<vector>
#include "linear_assignment.h"
#include "3rd_party/hungarianoper.h"
#include "definition/config.h"
#include "definition/data_type.h"
//#include"utils/util_logger.h"

#ifdef MAKEMD
#include<iostream>
#endif

using std::map;
using std::vector;

TRACHER_MATCHD
linear_assignment::matching_cascade(
    tracker *distance_metric,
    tracker::GATED_METRIC_FUNC distance_metric_func,
    float max_distance,
    int cascade_depth,
    vector<Track> &tracks,
    const DETECTION_REID_ROWS &detections,
    vector<int> &track_indices,
    vector<int> detection_indices)
{
    TRACHER_MATCHD res;
    //1.init unmatched_detections
    for (size_t i = 0; i < detections.size(); i++)
    {
        detection_indices.push_back(int(i));
    }
    vector<int> unmatched_detections;
    unmatched_detections.assign(detection_indices.begin(), detection_indices.end());

    res.matches.clear();
    vector<int> track_indices_l;
    map<int, int> matches_trackid;
    //2.cascade match
    for (int level = 0; level < cascade_depth; level++)
    {
        //No detections left;
        if (unmatched_detections.size() == 0)
            break;
        //(1).construct track_indices_l in this level
        track_indices_l.clear();
        for (int k : track_indices)
        {
            if (tracks[k].time_since_update == 1 + level)
                track_indices_l.push_back(k);
        }
        if (track_indices_l.size() == 0)
            continue; //Nothing to match at this level.
        //(2)compute cosine&mahalanobis distance
        //Hungarian match
        TRACHER_MATCHD tmp = min_cost_matching(
            distance_metric, distance_metric_func,
            max_distance, tracks, detections, track_indices_l,
            unmatched_detections);

        //(3)update res.matches & unmatched_detections
        unmatched_detections.assign(tmp.unmatched_detections.begin(), tmp.unmatched_detections.end());
        for (size_t i = 0; i < tmp.matches.size(); i++)
        {
            MATCH_DATA pa = tmp.matches[i];
            res.matches.push_back(pa);
            matches_trackid.insert(pa);
        }
    }
    //3.end for loop,update unmatched_detections & unmatched_tracks
    res.unmatched_detections.assign(unmatched_detections.begin(), unmatched_detections.end());
    for (size_t i = 0; i < track_indices.size(); i++)
    {
        int tid = track_indices[i];
        if (matches_trackid.find(tid) == matches_trackid.end())
            res.unmatched_tracks.push_back(tid);
    }
    return res;
}

TRACHER_MATCHD 
linear_assignment::min_cost_matching(tracker* distance_metric,
    tracker::GATED_METRIC_FUNC_NO_FEAT distance_metric_func,
    float max_distance,
    vector<Track>& tracks,
    const DETECTION_ROWS& detections,
    vector<int>& track_indices,
    vector<int>& detection_indices)
{
    TRACHER_MATCHD res;
    //1.Can't match
    if ((detection_indices.size() == 0) || (track_indices.size() == 0))
    {
        res.matches.clear();
        res.unmatched_tracks.assign(track_indices.begin(), track_indices.end());
        res.unmatched_detections.assign(detection_indices.begin(), detection_indices.end());
        return res;
    }

    //2.distance metric(compute)
    DYNAMICM cost_matrix = (distance_metric->*(distance_metric_func))(
        tracks, detections, track_indices, detection_indices);
#ifdef MAKEMD
    std::cout << "cost matrix get from distance_metric_func:\n";
    std::cout << cost_matrix << std::endl
        << std::endl;
#endif

    //3.distance select from max_distance
    for (int i = 0; i < cost_matrix.rows(); i++)
    {
        for (int j = 0; j < cost_matrix.cols(); j++)
        {
            float tmp = cost_matrix(i, j);
            if (tmp > max_distance)
                cost_matrix(i, j) = max_distance + 1e-5;
        }
    }
#ifdef MAKEMD
    std::cout << "cost matrix after dealing with max_distance: \n";
    std::cout << cost_matrix << std::endl
        << std::endl;
#endif

    //4.Hungarian match
    Eigen::Matrix<float, -1, 2, Eigen::RowMajor> indices =
        HungarianOper::Solve(cost_matrix);
#ifdef MAKEMD
    std::cout << "indices after HungarianOper: \n";
    std::cout << indices << std::endl
        << std::endl;
#endif

    //5.consturct ret
    res.matches.clear();
    res.unmatched_tracks.clear();
    res.unmatched_detections.clear();

    //consturct res.unmatched_detections
    for (size_t col = 0; col < detection_indices.size(); col++)
    {
        bool flag = false;
        for (int i = 0; i < indices.rows(); i++)
            if (indices(i, 1) == col)
            {
                flag = true;
                break;
            }
        if (flag == false)
            res.unmatched_detections.push_back(detection_indices[col]);
    }

    //consturct res.unmatched_tracks
    for (size_t row = 0; row < track_indices.size(); row++)
    {
        bool flag = false;
        for (int i = 0; i < indices.rows(); i++)
            if (indices(i, 0) == row)
            {
                flag = true;
                break;
            }
        if (flag == false)
            res.unmatched_tracks.push_back(track_indices[row]);
    }

    //construct res.matches & update  unmatched_tracks/unmatched_detections
    for (int i = 0; i < indices.rows(); i++)
    {
        int row = indices(i, 0);
        int col = indices(i, 1);

        int track_idx = track_indices[row];
        int detection_idx = detection_indices[col];
        if (cost_matrix(row, col) > max_distance)
        {
            res.unmatched_tracks.push_back(track_idx);
            res.unmatched_detections.push_back(detection_idx);
        }
        else
            res.matches.push_back(std::make_pair(track_idx, detection_idx));
    }
    return res;
}


TRACHER_MATCHD
linear_assignment::min_cost_matching(tracker *distance_metric,
                                     tracker::GATED_METRIC_FUNC distance_metric_func,
                                     float max_distance,
                                     vector<Track> &tracks,
                                     const DETECTION_REID_ROWS &detections,
                                     vector<int> &track_indices,
                                     vector<int> &detection_indices)
{
    TRACHER_MATCHD res;
    //1.Can't match
    if ((detection_indices.size() == 0) || (track_indices.size() == 0))
    {
        res.matches.clear();
        res.unmatched_tracks.assign(track_indices.begin(), track_indices.end());
        res.unmatched_detections.assign(detection_indices.begin(), detection_indices.end());
        return res;
    }

    //2.distance metric(compute)
    DYNAMICM cost_matrix = (distance_metric->*(distance_metric_func))(
        tracks, detections, track_indices, detection_indices);
#ifdef MAKEMD
    std::cout << "cost matrix get from distance_metric_func:\n";
    std::cout << cost_matrix << std::endl
              << std::endl;
#endif

    //3.distance select from max_distance
    for (int i = 0; i < cost_matrix.rows(); i++)
    {
        for (int j = 0; j < cost_matrix.cols(); j++)
        {
            float tmp = cost_matrix(i, j);
            if (tmp > max_distance)
                cost_matrix(i, j) = max_distance + 1e-5;
        }
    }
#ifdef MAKEMD
    std::cout << "cost matrix after dealing with max_distance: \n";
    std::cout << cost_matrix << std::endl
              << std::endl;
#endif

    //4.Hungarian match
    Eigen::Matrix<float, -1, 2, Eigen::RowMajor> indices =
        HungarianOper::Solve(cost_matrix);
#ifdef MAKEMD
    std::cout << "indices after HungarianOper: \n";
    std::cout << indices << std::endl
              << std::endl;
#endif

    //5.consturct ret
    res.matches.clear();
    res.unmatched_tracks.clear();
    res.unmatched_detections.clear();

    //consturct res.unmatched_detections
    for (size_t col = 0; col < detection_indices.size(); col++)
    {
        bool flag = false;
        for (int i = 0; i < indices.rows(); i++)
            if (indices(i, 1) == col)
            {
                flag = true;
                break;
            }
        if (flag == false)
            res.unmatched_detections.push_back(detection_indices[col]);
    }

    //consturct res.unmatched_tracks
    for (size_t row = 0; row < track_indices.size(); row++)
    {
        bool flag = false;
        for (int i = 0; i < indices.rows(); i++)
            if (indices(i, 0) == row)
            {
                flag = true;
                break;
            }
        if (flag == false)
            res.unmatched_tracks.push_back(track_indices[row]);
    }

    //construct res.matches & update  unmatched_tracks/unmatched_detections
    for (int i = 0; i < indices.rows(); i++)
    {
        int row = indices(i, 0);
        int col = indices(i, 1);

        int track_idx = track_indices[row];
        int detection_idx = detection_indices[col];
        if (cost_matrix(row, col) > max_distance)
        {
            res.unmatched_tracks.push_back(track_idx);
            res.unmatched_detections.push_back(detection_idx);
        }
        else
            res.matches.push_back(std::make_pair(track_idx, detection_idx));
    }
    return res;
}

DYNAMICM
linear_assignment::gate_cost_matrix(
    DeepSortKalmanFilter *kf,
    DYNAMICM &cost_matrix,
    vector<Track> &tracks,
    const DETECTION_REID_ROWS &detections,
    const vector<int> &track_indices,
    const vector<int> &detection_indices,
    float gated_cost, bool only_position)
{
    int gating_dim = (only_position == true ? 2 : 4);
    double gating_threshold = DeepSortKalmanFilter::chi2inv95[gating_dim];
    vector<DETECTBOX> measurements;
    for (int i : detection_indices)
    {
        DETECTION_REID_ROW t = detections[i];
        measurements.push_back(t.to_xyah());
    }
    for (size_t i = 0; i < track_indices.size(); i++)
    {
        Track &track = tracks[track_indices[i]];
        Eigen::Matrix<float, 1, -1> gating_distance = kf->gating_distance(
            track.mean, track.covariance, measurements, only_position);
        for (int j = 0; j < gating_distance.cols(); j++)
        {
            if (gating_distance(0, j) > gating_threshold)
                cost_matrix(i, j) = gated_cost;
        }
    }
    return cost_matrix;
}
