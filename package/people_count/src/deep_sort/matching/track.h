#ifndef TRACK_H
#define TRACK_H

#include "definition/data_type.h"

#include "kalmanfilter.h"

class Track
{
    /*"""
    A single target track with state space `(x, y, a, h)` and associated
    velocities, where `(x, y)` is the center of the bounding box, `a` is the
    aspect ratio and `h` is the height.

    Parameters
    ----------
    mean : ndarray
        Mean vector of the initial state distribution.
    covariance : ndarray
        Covariance matrix of the initial state distribution.
    track_id : int
        A unique track identifier.
    n_init : int
        Number of consecutive detections before the track is confirmed. The
        track state is set to `Deleted` if a miss occurs within the first
        `n_init` frames.
    max_age : int
        The maximum number of consecutive misses before the track state is
        set to `Deleted`.
    feature : Optional[ndarray]
        Feature vector of the detection this track originates from. If not None,
        this feature is added to the `features` cache.

    Attributes
    ----------
    mean : ndarray
        Mean vector of the initial state distribution.
    covariance : ndarray
        Covariance matrix of the initial state distribution.
    track_id : int
        A unique track identifier.
    hits : int
        Total number of measurement updates.
    age : int
        Total number of frames since first occurance.
    time_since_update : int
        Total number of frames since last measurement update.
    state : TrackState
        The current track state.
    features : List[ndarray]
        A cache of features. On each measurement update, the associated feature
        vector is added to this list.

    """*/
    enum TrackState
    {
        Tentative = 1,
        Confirmed,
        Deleted
    };

public:
    Track(KAL_MEAN &mean, KAL_COVA &covariance, int track_id,
          int n_init, int max_age, const FEATURE &feature);
    Track(KAL_MEAN& mean, KAL_COVA& covariance, int track_id, 
          int n_init,int max_hits);

    ~Track();
    void predit(DeepSortKalmanFilter *kf);
    void update(DeepSortKalmanFilter* const kf, const DETECTION_ROW& detection,DEEP_SORT_RET& ret);
    void update(DeepSortKalmanFilter *const kf, const DETECTION_REID_ROW &detection);
    void mark_missed();
    void mark_missed(DEEP_SORT_RET& ret);
    bool is_confirmed();
    bool is_deleted();
    bool is_tentative();
    DETECTBOX to_tlwh();
    
    int track_id;
    int time_since_update;
    int hits;
    int age;
    TrackState state;
    int _n_init;
    int _max_age;
    int max_hits;
    
    DETECTBOX det_bbox;
    FEATURESS features;
    KAL_MEAN mean;
    KAL_COVA covariance;

    

private:
    void featuresAppendOne(const FEATURE &f);
};

#endif // TRACK_H
