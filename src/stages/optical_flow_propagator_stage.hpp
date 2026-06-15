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
        average_transform_  = cv::Mat::eye(3, 3, CV_64F);
        accumulated_count_  = 0;
        propagation_count_  = 0;
        propagation_center_ = cv::Point2f(-1, -1);
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
            prev_gray_ = std::move(gray);

            phase_ = Phase::ACCUMULATING;
            accumulated_transforms_.clear();
            accumulated_count_  = 0;
            propagation_center_ = cv::Point2f(-1, -1);
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
            do_propagate(ctx);
            return;
        }
    }

  private:
    enum class Phase { IDLE, ACCUMULATING, PROPAGATING };

    std::shared_ptr<std::atomic<bool>> orb_active_;

    int win_size_;
    int pyr_levels_;
    int min_tracking_pts_;
    int max_propagation_frames_;
    int window_frames_;

    Phase phase_ = Phase::IDLE;

    cv::Mat                  prev_gray_;
    std::vector<cv::Point2f> prev_pts_;

    std::vector<cv::Mat> accumulated_transforms_;
    int                  accumulated_count_ = 0;

    cv::Mat     average_transform_  = cv::Mat::eye(3, 3, CV_64F);
    int         propagation_count_  = 0;
    cv::Point2f propagation_center_ = cv::Point2f(-1, -1);

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
                                                   cv::RANSAC, 3.0, 5, 0.98, 10);
        cv::Mat T = cv::Mat::eye(3, 3, CV_64F);
        if (!M23.empty())
            M23.copyTo(T.rowRange(0, 2));
        accumulated_transforms_.push_back(T);
        ++accumulated_count_;

        cv::Mat warp   = T.inv();
        cv::Mat warp23;
        warp.rowRange(0, 2).convertTo(warp23, CV_32F);
        cv::Mat warped;
        cv::warpAffine(ctx->frame, warped, warp23, ctx->frame.size(),
                       cv::INTER_NEAREST, cv::BORDER_REPLICATE);
        ctx->frame = std::move(warped);

        ctx->ransac_result.emplace();
        warp.convertTo(ctx->ransac_result->homography, CV_32F);

        auto &ofr        = ctx->optical_flow_result.emplace();
        ofr.points_prev  = prev_filtered;
        ofr.points_curr  = curr_filtered;
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

            // Compute center from final tracked points before discarding them
            if (!prev_pts_.empty()) {
                cv::Point2f center(0, 0);
                for (const auto &pt : prev_pts_)
                    center += pt;
                propagation_center_ = center * (1.0f / static_cast<float>(prev_pts_.size()));
            }
            prev_pts_.clear();
            prev_gray_.release();

            phase_             = Phase::PROPAGATING;
            propagation_count_ = 0;
        }

        ctx->flags.has_inliers = true;
    }

    void do_propagate(std::shared_ptr<FrameContext> ctx) {
        if (++propagation_count_ > max_propagation_frames_) {
            orb_active_->store(true);
            phase_              = Phase::IDLE;
            propagation_center_ = cv::Point2f(-1, -1);
            apply_identity(ctx);
            return;
        }

        if (propagation_center_.x < 0 ||
            propagation_center_.x >= ctx->frame.cols ||
            propagation_center_.y < 0 ||
            propagation_center_.y >= ctx->frame.rows) {
            orb_active_->store(true);
            phase_              = Phase::IDLE;
            propagation_center_ = cv::Point2f(-1, -1);
            apply_identity(ctx);
            return;
        }

        std::vector<cv::Point2f> prev     = { propagation_center_ };
        std::vector<cv::Point2f> estimated = { propagation_center_ };
        cv::perspectiveTransform(estimated, estimated, average_transform_);

        auto &ofr            = ctx->optical_flow_result.emplace();
        ofr.points_prev      = prev;
        ofr.points_curr      = estimated;
        ofr.suggested_center = estimated[0];

        propagation_center_ = estimated[0];

        ctx->ransac_result.emplace();
        ctx->ransac_result->homography = cv::Mat::eye(3, 3, CV_32F);
        ctx->flags.has_inliers         = true;
    }

    void apply_identity(std::shared_ptr<FrameContext> ctx) {
        ctx->ransac_result.emplace();
        ctx->ransac_result->homography = cv::Mat::eye(3, 3, CV_32F);
        ctx->flags.has_inliers         = true;
    }
};