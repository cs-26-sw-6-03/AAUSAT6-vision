#pragma once

#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include <atomic>
#include <memory>
#include <opencv2/opencv.hpp>

class OpticalFlowStage : public ThreadedStage {
  public:
    OpticalFlowStage(std::shared_ptr<Router> router, const Config &cfg,
                     std::shared_ptr<std::atomic<bool>> orb_active, int cpu_affinity = -1)
        : ThreadedStage("optical_flow", std::move(router), cfg.get<int>("pipeline.queue_size", 32), cpu_affinity),
          orb_active_(std::move(orb_active)),
          min_tracking_pts_(cfg.get<int>("optical_flow.min_tracking_pts", 200)),
          win_size_(cfg.get<int>("optical_flow.win_size", 21)),
          pyr_levels_(cfg.get<int>("optical_flow.pyr_levels", 3)) {
    }

    void init() override {
        prev_gray_.release();
        prev_pts_.clear();
        frame_idx_ = 0;
    }

    void process(std::shared_ptr<FrameContext> ctx) override {
        cv::Mat gray;
        cv::cvtColor(ctx->frame, gray, cv::COLOR_BGR2GRAY);

        // Propagate one-frame-delayed reseeding signal to the stabilizer
        if (reset_trajectory_next_) {
            ctx->optical_flow_result->tracking_reseeded = true;
            reset_trajectory_next_                      = false;
        }

        // Seed tracking points from ORB keypoints whenever ORB ran
        bool just_seeded = false;
        if (ctx->orb_result.has_value() && !ctx->orb_result->keypoints.empty()) {
            if (ctx->orb_result->has_matches && ctx->matching_result.has_value()) {
                //match was found, only track matched keypoints, ORB goes to passive
                prev_pts_.clear();
                for (const auto &kp : ctx->orb_result->keypoints)
                    prev_pts_.push_back(kp.pt);
                orb_active_->store(false);
                just_seeded = true;
                ctx->optical_flow_result->tracking_just_seeded = true;
                reset_trajectory_next_ = true;
            } else if (prev_pts_.empty()) {
                // no db match, no tracking, just use all keypoints as seeds and keep ORB active for stabiliser
                for (const auto &kp : ctx->orb_result->keypoints)
                    prev_pts_.push_back(kp.pt);
                just_seeded = true;
                ctx->optical_flow_result->tracking_just_seeded = true;
                reset_trajectory_next_ = true;
            }
        }

        ctx->flags.has_keypoints   = false;
        ctx->flags.skip_processing = true;

        //Just store gray on first frame to initialize prev_gray_ without doing any tracking
        if (prev_gray_.empty() || prev_pts_.empty() || just_seeded) {
            cv::swap(prev_gray_, gray);
            ++frame_idx_;
            return;
        }

        // Track points from previous frame to current frame
        std::vector<cv::Point2f> curr_pts;
        std::vector<uchar>       status;
        std::vector<float>       err;
        cv::calcOpticalFlowPyrLK(prev_gray_, gray, prev_pts_, curr_pts, status, err,
                                 cv::Size(win_size_, win_size_), pyr_levels_,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), 0);

        auto &result = ctx->optical_flow_result.emplace();

        // Filter to successfully tracked points
        std::vector<cv::Point2f> prevFiltered, currFiltered;
        for (size_t i = 0; i < status.size(); ++i) {
            if (status[i]) {
                prevFiltered.push_back(prev_pts_[i]);
                currFiltered.push_back(curr_pts[i]);
            }
        }

        result.points_prev    = prevFiltered;
        result.points_curr    = currFiltered;
        result.status         = status;
        result.tracking_score = prev_pts_.empty() ? 0.0f
                                                  : static_cast<float>(currFiltered.size()) / static_cast<float>(prev_pts_.size());

        if (static_cast<int>(currFiltered.size()) < min_tracking_pts_) {
            // Tracking lost — signal ORB to reactivate, clear points
            orb_active_->store(true);
            prev_pts_.clear();
        } else {
            // Update tracked points for next frame
            prev_pts_ = currFiltered;

            if (!currFiltered.empty()) {
                cv::Point2f center(0, 0);
                for (const auto &pt : currFiltered)
                    center += pt;
                result.suggested_center = center * (1.0f / currFiltered.size());
            }
        }

        cv::swap(prev_gray_, gray);
        ++frame_idx_;
    }

  private:
    std::shared_ptr<std::atomic<bool>> orb_active_;

    cv::Mat                  prev_gray_;
    std::vector<cv::Point2f> prev_pts_;
    size_t                   frame_idx_ = 0;
    int                      min_tracking_pts_;
    int                      win_size_;
    int                      pyr_levels_;
    bool                     reset_trajectory_next_ = false;
};
