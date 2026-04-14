#pragma once

#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

// AffineEstimatorStage — computes the per-frame stabilization warp.
//
// Stage 1 of the two-stage stabilizer. Runs on its own thread.
//
// Uses cv::estimateAffinePartial2D (4-DOF: tx, ty, rotation, uniform scale)
// with a single RANSAC call, then applies EMA smoothing to the accumulated
// trajectory and stores the correction warp in ransac_result->homography.
// Does NOT touch ctx->frame — that is deferred to WarpApplyStage so that
// warp estimation and warp application can run in parallel across frames.
//
// Routing:
//   Normal path  → sets has_warp = true  → routed to "warp_apply"
//   Identity path → sets has_inliers = true → routed directly to "pose"
//     (identity frames don't need warp application)
class AffineEstimatorStage : public ThreadedStage
{
public:
    AffineEstimatorStage(std::shared_ptr<Router> router, const Config& cfg, int cpu_affinity = -1)
        : ThreadedStage("ransac", std::move(router),
                        cfg.get<int>("pipeline.queue_size", 32), cpu_affinity)
    {
        reproj_threshold_ = cfg.get<float> ("stabilizer.reproj_threshold", 3.0f);
        min_inliers_      = cfg.get<int>   ("stabilizer.min_inliers",      6);
        alpha_            = cfg.get<double>("stabilizer.smoothing",         0.05);
    }

    void init() override
    {
        T_smooth_    = cv::Mat::eye(3, 3, CV_64F);
        T_prev_      = cv::Mat::eye(3, 3, CV_64F);
        initialized_ = false;
        frame_idx_   = 0;
    }

    void process(std::shared_ptr<FrameContext> ctx) override
    {
        const bool have_lk = ctx->optical_flow_result.has_value() &&
                             !ctx->optical_flow_result->points_prev.empty();

        const bool do_reset = ctx->optical_flow_result.has_value() &&
                              (ctx->optical_flow_result->tracking_just_seeded ||
                               ctx->optical_flow_result->tracking_reseeded);

        if (do_reset || !initialized_) {
            T_smooth_    = cv::Mat::eye(3, 3, CV_64F);
            T_prev_      = cv::Mat::eye(3, 3, CV_64F);
            initialized_ = true;
            emit_identity(ctx);
            ++frame_idx_;
            return;
        }

        if (!have_lk) {
            emit_identity(ctx);
            ++frame_idx_;
            return;
        }

        const auto& pts_prev = ctx->optical_flow_result->points_prev;
        const auto& pts_curr = ctx->optical_flow_result->points_curr;

        if ((int)pts_prev.size() < min_inliers_) {
            std::cerr << "[AffineEstimatorStage] too few LK points ("
                      << pts_prev.size() << ") at frame " << frame_idx_
                      << " — using identity\n";
            emit_identity(ctx);
            ++frame_idx_;
            return;
        }

        // Single-pass 4-DOF partial affine estimation (tx, ty, rotation, uniform scale)
        cv::Mat M23 = cv::estimateAffinePartial2D(
            pts_prev, pts_curr,
            cv::noArray(), cv::RANSAC, reproj_threshold_,
            2000, 0.995, 10);

        cv::Mat M33 = cv::Mat::eye(3, 3, CV_64F);
        if (!M23.empty())
            M23.copyTo(M33.rowRange(0, 2));
        else
            std::cerr << "[AffineEstimatorStage] estimation failed at frame "
                      << frame_idx_ << " — using identity\n";

        // Accumulate trajectory: T_curr = M_inter * T_prev
        cv::Mat T_curr = M33 * T_prev_;

        // EMA smoothing: T_smooth = alpha*T_curr + (1-alpha)*T_smooth
        T_smooth_ = alpha_ * T_curr + (1.0 - alpha_) * T_smooth_;

        // Correction warp: bring raw frame onto the smoothed trajectory
        // Stored as CV_32F — warpAffine works in float natively, avoids per-frame conversion
        cv::Mat warp33 = T_smooth_ * T_curr.inv();

        ctx->ransac_result.emplace();
        warp33.convertTo(ctx->ransac_result->homography, CV_32F);
        ctx->flags.has_warp = true;   // routes to WarpApplyStage

        T_prev_ = T_curr.clone();
        ++frame_idx_;
    }

private:
    float  reproj_threshold_ = 3.0f;
    int    min_inliers_      = 6;
    double alpha_            = 0.05;

    cv::Mat     T_smooth_;
    cv::Mat     T_prev_;
    bool        initialized_ = false;
    std::size_t frame_idx_   = 0;

    // Identity path: store identity homography and skip warp_apply entirely
    void emit_identity(std::shared_ptr<FrameContext>& ctx)
    {
        ctx->ransac_result.emplace();
        ctx->ransac_result->homography = cv::Mat::eye(3, 3, CV_32F);
        ctx->flags.has_inliers = true;  // bypass warp_apply, go straight to pose
    }
};
