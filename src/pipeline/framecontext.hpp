#pragma once

#include <chrono>
#include <cstdint>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

struct RoutingFlags {
    bool from_input      = false;
    bool has_keypoints   = false; 
    bool skip_processing = false;
    bool has_warp        = false; 
    bool has_pose        = false; 
    bool has_inliers     = false; 
    bool done            = false; 
    bool drop_frame      = false;
    bool needs_redetect = false;

};
struct OpticalFlowResult {
    std::vector<cv::Point2f> points_prev;
    std::vector<cv::Point2f> points_curr;
    std::vector<uchar>       status; // Per-point tracking status
    float                    tracking_score = 0.0f;
    cv::Point2f              suggested_center; // Fraction of points tracked
    bool                     tracking_just_seeded = false;
    bool                     tracking_reseeded    = false;
};

struct OrbResult {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat                   descriptors; // See docs for cv::Mat descriptor format
    std::vector<cv::KeyPoint> object_keypoints;
    cv::Mat                   object_descriptors;
    cv::Size                  object_size; // Reference image dimensions (for projecting center through H)
    bool                      has_matches = false;
};

struct MatchingResult {
    std::vector<cv::DMatch> matches; 
};

struct RansacResult {
    cv::Mat                 homography; // 3x3, empty if estimation failed
    std::vector<cv::DMatch> inliers;
    std::vector<cv::DMatch> outliers;
    double                  reprojection_error = 0.0;
    int                     iterations_used    = 0;
};

struct PoseResult {
    cv::Mat     rotation;    // 3x3 rotation matrix
    cv::Mat     translation; // 3x1 translation vector
    bool        valid = false;
    cv::Point2f center;
    float       confidence = 0.f;
};

struct StageTiming {
    std::chrono::steady_clock::time_point queue_enter{};
    std::chrono::steady_clock::time_point queue_dequeue{};
    std::chrono::steady_clock::time_point process_start{};
    std::chrono::steady_clock::time_point process_end{};
};

struct FrameTelemetry {
    std::unordered_map<std::string, StageTiming> per_stage;
    bool                                         logged = false;
};

struct FrameContext {
    uint64_t                              frame_id;
    std::string                           source_id; // Camera / stream identifier
    std::chrono::steady_clock::time_point timestamp; // Frame capture time

    cv::Mat frame; // Current frame (BGR)

    std::optional<OpticalFlowResult> optical_flow_result;
    std::optional<OrbResult>         orb_result;
    std::optional<MatchingResult>    matching_result;
    std::optional<RansacResult>      ransac_result;
    std::optional<PoseResult>        pose_result;

    RoutingFlags flags;

    std::optional<cv::Mat> debug_frame;

    FrameTelemetry telemetry;
};