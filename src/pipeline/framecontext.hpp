#pragma once

/* 
 * framecontext.hpp
 *
 * Struct to pass context information between pipeline stages.
 * Carries the frame, results so far, and flags for controlling pipeline flow.
 */

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <vector>
#include <optional>
#include <chrono>
#include <cstdint>
#include <string>

// Routing flags: set by stages to control where the frame goes next.
// Pipeline order: capture -> orb -> optical_flow -> pose -> ransac -> output
struct RoutingFlags {
    bool from_input       = false;    // Fresh from capture              -> route to "orb"
    bool has_keypoints    = false;    // ORB done (active or passive)    -> route to "optical_flow"
    bool skip_processing  = false;    // Optical flow done               -> route to "pose"
    bool has_pose         = false;    // Pose done                       -> route to "ransac"
    bool has_inliers      = false;    // RANSAC done                     -> route to "output"
    bool drop_frame       = false;    // Frame is unusable               -> discard

    // Informational only (not used for routing)
    bool needs_redetect      = false;    // Optical flow lost tracking — ORB switched to active
    bool has_matches         = false;    // ORB active mode found DB matches
    bool tracking_reseeded    = false;    // stabilizer should reset trajectory this frame (N+1)
    bool tracking_just_seeded = false;   // ORB seeded this frame — stabilizer must skip H_inter (N)
};

// Optical flow result struct
struct OpticalFlowResult {
    std::vector<cv::Point2f> points_prev;
    std::vector<cv::Point2f> points_curr;
    std::vector<uchar>       status;                   // Per-point tracking status
    float                    tracking_score = 0.0f;
    cv::Point2f              suggested_center;    // Fraction of points tracked
};

// Oriented 'Features from Accelerated Segment Test (FAST)' and Rotated 'Binary Robust Independent Elementary Features (BRIEF)' (ORB) result
// ORB result struct
struct OrbResult {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat                   descriptors; // See docs for cv::Mat descriptor format
    std::vector<cv::KeyPoint> object_keypoints;
    cv::Mat                   object_descriptors;
    cv::Size                  object_size;  // Reference image dimensions (for projecting center through H)
};

// Matching result struct
// For marches between two frames, or between frame and map points
struct MatchingResult {
    std::vector<cv::DMatch> matches;        // After ratio test
    //std::vector<cv::DMatch> raw_matches;    // Before ratio test, for inspection
};

// RANSAC result stuct
// For homography estimation between two frames, or between frame and map points
struct RansacResult {
    cv::Mat                 homography;    // 3x3, empty if estimation failed
    std::vector<cv::DMatch> inliers;
    std::vector<cv::DMatch> outliers;
    double                  reprojection_error = 0.0;
    int                     iterations_used    = 0;
};

// Pose result struct
// for pose estimation between two frames, or between frame and map points
struct PoseResult {
    cv::Mat rotation;       // 3x3 rotation matrix
    cv::Mat translation;    // 3x1 translation vector
    bool    valid = false;
    cv::Point2f center;
    float confidence = 0.f;
};


struct FrameContext {
    // The frames identity
    uint64_t frame_id;
    std::string source_id;                              // Camera / stream identifier
    std::chrono::steady_clock::time_point timestamp;    // Frame capture time

    // The raw frame data
    cv::Mat frame;         // Current frame (BGR)
    //cv::Mat frame_prev;    // Previous frame, if needed by flow


    // Stage results
    std::optional<OpticalFlowResult> optical_flow_result;
    std::optional<OrbResult>         orb_result;
    std::optional<MatchingResult>    matching_result;
    std::optional<RansacResult>      ransac_result;
    std::optional<PoseResult>        pose_result;

    // Routing 
    RoutingFlags flags; // See above

    // Debug / Inspection frame data (annotated frame for visualisation)
    std::optional<cv::Mat> debug_frame;
};