#pragma once

#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include <atomic>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

class OpticalFlowPropagatorStage : public ThreadedStage {
  public:
    OpticalFlowPropagatorStage(std::shared_ptr<Router> router, const Config &cfg,
                               std::shared_ptr<std::atomic<bool>> orb_active, int cpu_affinity = -1)
        : ThreadedStage("optical_flow_propagator", std::move(router), cfg.get<int>("pipeline.queue_size", 32), cpu_affinity),
          orb_active_(std::move(orb_active)),
          window_frames_(cfg.get<int>("optical_flow_propagator.window_frames", 30)),
          win_size_(cfg.get<int>("optical_flow_propagator.win_size", 41)),
          pyr_levels_(cfg.get<int>("optical_flow_propagator.pyr_levels", 5)),
          min_tracking_pts_(cfg.get<int>("optical_flow_propagator.min_tracking_pts", 20)),
          max_propagation_frames_(cfg.get<int>("optical_flow_propagator.max_propagation_frames", 300)) {}

    void init() override {
        phase_ = Phase::IDLE;
        prev_gray_.release();
        prev_pts_.clear();
        accumulated_transforms_.clear();
        average_transform_ = cv::Mat::eye(3, 3, CV_64F);
        accumulated_count_ = 0;
        propagation_count_ = 0;
        orb_active_->store(true);
    }

    void process(std::shared_ptr<FrameContext> ctx) override {
        cv::Mat gray;
        cv::cvtColor(ctx->frame, gray, cv::COLOR_BGR2GRAY);

        bool fresh_detection = ctx->orb_result.has_value() &&
                               ctx->orb_result->has_matches &&
                               ctx->matching_result.has_value() &&
                               !ctx->matching_result->matches.empty();

        if (fresh_detection) {
            prev_pts_.clear();
            for (const auto &m : ctx->matching_result->matches)
                prev_pts_.push_back(ctx->orb_result->keypoints[m.queryIdx].pt);
            prev_gray_ = gray.clone();

            phase_ = Phase::ACCUMULATING;
            accumulated_transforms_.clear();
            accumulated_count_ = 0;
            orb_active_->store(false);

            apply_identity(ctx);
            return;
        }

        switch (phase_) {
        case Phase::IDLE:
            apply_identity(ctx);
            return;
        case Phase::ACCUMULATING:
            do_accumulate(gray, ctx);
            return;
        case Phase::PROPAGATING:
            do_propagate(gray, ctx);
            return;
        }
    }

  private:
    enum class Phase { IDLE, ACCUMULATING, PROPAGATING };

    std::shared_ptr<std::atomic<bool>> orb_active_;

    int window_frames_;
    int win_size_;
    int pyr_levels_;
    int min_tracking_pts_;
    int max_propagation_frames_;

    Phase phase_ = Phase::IDLE;

    cv::Mat                  prev_gray_;
    std::vector<cv::Point2f> prev_pts_;

    std::vector<cv::Mat> accumulated_transforms_;
    int                  accumulated_count_ = 0;

    cv::Mat average_transform_ = cv::Mat::eye(3, 3, CV_64F);
    int     propagation_count_ = 0;

    void do_accumulate(const cv::Mat &gray, std::shared_ptr<FrameContext> ctx) {
        if (prev_pts_.empty() || prev_gray_.empty()) {
            apply_identity(ctx);
            return;
        }

        std::vector<cv::Point2f> curr_pts;
        std::vector<uchar>       status;
        std::vector<float>       err;
        cv::calcOpticalFlowPyrLK(prev_gray_, gray, prev_pts_, curr_pts, status, err,
                                 cv::Size(win_size_, win_size_), pyr_levels_,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), 0);

        std::vector<cv::Point2f> prev_filtered, curr_filtered;
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                prev_filtered.push_back(prev_pts_[i]);
                curr_filtered.push_back(curr_pts[i]);
            }
        }

        if (static_cast<int>(curr_filtered.size()) < min_tracking_pts_) {
            orb_active_->store(true);
            phase_ = Phase::IDLE;
            prev_pts_.clear();
            apply_identity(ctx);
            return;
        }

        cv::Mat M23 = cv::estimateAffinePartial2D(prev_filtered, curr_filtered, cv::noArray(),
                                                   cv::RANSAC, 3.0, 2000, 0.995, 10);
        cv::Mat T = cv::Mat::eye(3, 3, CV_64F);
        if (!M23.empty())
            M23.copyTo(T.rowRange(0, 2));
        accumulated_transforms_.push_back(T);
        ++accumulated_count_;

        cv::Mat warp    = T.inv();
        cv::Mat warp23  = warp.rowRange(0, 2);
        ctx->frame      = ctx->frame.clone();
        cv::warpAffine(ctx->frame, ctx->frame, warp23, ctx->frame.size(),
                       cv::INTER_NEAREST, cv::BORDER_REPLICATE);

        ctx->ransac_result.emplace();
        warp.convertTo(ctx->ransac_result->homography, CV_32F);

        auto &ofr = ctx->optical_flow_result.emplace();
        ofr.points_prev = prev_filtered;
        ofr.points_curr = curr_filtered;
        if (!curr_filtered.empty()) {
            cv::Point2f center(0, 0);
            for (const auto &pt : curr_filtered)
                center += pt;
            ofr.suggested_center = center * (1.0f / static_cast<float>(curr_filtered.size()));
        }

        prev_pts_  = curr_filtered;
        prev_gray_ = gray.clone();

        if (accumulated_count_ >= window_frames_) {
            average_transform_ = cv::Mat::zeros(3, 3, CV_64F);
            for (const auto &m : accumulated_transforms_)
                average_transform_ += m;
            average_transform_ /= static_cast<double>(accumulated_transforms_.size());

            phase_             = Phase::PROPAGATING;
            propagation_count_ = 0;
        }

        ctx->flags.has_inliers = true;
    }

    void do_propagate(const cv::Mat &, std::shared_ptr<FrameContext> ctx) {
        if (++propagation_count_ > max_propagation_frames_) {
            orb_active_->store(true);
            phase_ = Phase::IDLE;
            prev_pts_.clear();
            apply_identity(ctx);
            return;
        }

        std::vector<cv::Point2f> estimated_pts;
        if (!prev_pts_.empty())
            cv::perspectiveTransform(prev_pts_, estimated_pts, average_transform_);

        int in_bounds = 0;
        for (const auto &pt : estimated_pts) {
            if (pt.x >= 0 && pt.x < ctx->frame.cols &&
                pt.y >= 0 && pt.y < ctx->frame.rows)
                ++in_bounds;
        }

        if (static_cast<int>(estimated_pts.size()) < min_tracking_pts_ ||
            in_bounds < min_tracking_pts_) {
            orb_active_->store(true);
            phase_ = Phase::IDLE;
            prev_pts_.clear();
            apply_identity(ctx);
            return;
        }

        cv::Mat warp   = average_transform_.inv();
        cv::Mat warp23 = warp.rowRange(0, 2);
        ctx->frame     = ctx->frame.clone();
        cv::warpAffine(ctx->frame, ctx->frame, warp23, ctx->frame.size(),
                       cv::INTER_NEAREST, cv::BORDER_REPLICATE);

        ctx->ransac_result.emplace();
        warp.convertTo(ctx->ransac_result->homography, CV_32F);

        auto &ofr = ctx->optical_flow_result.emplace();
        ofr.points_prev = prev_pts_;
        ofr.points_curr = estimated_pts;
        if (!estimated_pts.empty()) {
            cv::Point2f center(0, 0);
            for (const auto &pt : estimated_pts)
                center += pt;
            ofr.suggested_center = center * (1.0f / static_cast<float>(estimated_pts.size()));
        }

        prev_pts_ = estimated_pts;

        ctx->flags.has_inliers = true;
    }

    void apply_identity(std::shared_ptr<FrameContext> ctx) {
        ctx->ransac_result.emplace();
        ctx->ransac_result->homography = cv::Mat::eye(3, 3, CV_32F);
        ctx->flags.has_inliers         = true;
    }
};
