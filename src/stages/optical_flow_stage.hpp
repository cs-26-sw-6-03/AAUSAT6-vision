#pragma once

#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include <opencv2/opencv.hpp>
#include <atomic>
#include <memory>

class OpticalFlowStage : public ThreadedStage
{
public:
    OpticalFlowStage(std::shared_ptr<Router> router, const Config &cfg,
                     std::shared_ptr<std::atomic<bool>> orb_active, int cpu_affinity = -1)
        : ThreadedStage("optical_flow", std::move(router), cfg.get<int>("pipeline.queue_size", 32), cpu_affinity)
        , orb_active_(std::move(orb_active))
        , min_tracking_pts_(cfg.get<int>("optical_flow.min_tracking_pts", 200))
        , win_size_(cfg.get<int>("optical_flow.win_size", 21))
        , pyr_levels_(cfg.get<int>("optical_flow.pyr_levels", 3))
    {
    }

    void init() override
    {
        prev_gray_.release();
        prev_pts_.clear();
        frame_idx_ = 0;
    }

    void process(std::shared_ptr<FrameContext> ctx) override
    {
        cv::Mat gray;
        cv::cvtColor(ctx->frame, gray, cv::COLOR_BGR2GRAY);

        // Propagate one-frame-delayed reseeding signal to the stabilizer
        if (reset_trajectory_next_) {
            ctx->flags.tracking_reseeded = true;
            reset_trajectory_next_ = false;
        }

        // Seed tracking points from ORB keypoints whenever ORB ran
        bool just_seeded = false;
        if (ctx->orb_result.has_value() && !ctx->orb_result->keypoints.empty())
        {
            if (ctx->flags.has_matches && ctx->matching_result.has_value())
            {
                // DB match found: seed from matched keypoints only, switch ORB to passive
                prev_pts_.clear();
                for (const auto& m : ctx->matching_result->matches)
                    prev_pts_.push_back(ctx->orb_result->keypoints[m.queryIdx].pt);
                orb_active_->store(false);
                just_seeded = true;
                reset_trajectory_next_ = true;
            }
            else if (prev_pts_.empty())
            {
                // No DB match yet and no active tracking: seed from all detected keypoints
                // so the stabilizer can run while ORB keeps searching for a reference match
                for (const auto& kp : ctx->orb_result->keypoints)
                    prev_pts_.push_back(kp.pt);
                // Keep ORB active — it must keep searching for a DB match
                just_seeded = true;
                reset_trajectory_next_ = true;
            }
        }

        ctx->flags.has_keypoints   = false;
        ctx->flags.skip_processing = true;

        // On seeding frames, store current gray and skip LK — the seeded positions are
        // in the current frame so tracking must start from the next frame to be correct.
        // Also skip if there is no previous frame or no points to track.
        if (prev_gray_.empty() || prev_pts_.empty() || just_seeded)
        {
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

        auto& result = ctx->optical_flow_result.emplace();

        // Filter to successfully tracked points
        std::vector<cv::Point2f> prevFiltered, currFiltered;
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                prevFiltered.push_back(prev_pts_[i]);
                currFiltered.push_back(curr_pts[i]);
            }
        }

        result.points_prev     = prevFiltered;
        result.points_curr     = currFiltered;
        result.status          = status;
        result.tracking_score  = prev_pts_.empty() ? 0.0f
            : static_cast<float>(currFiltered.size()) / static_cast<float>(prev_pts_.size());

        if (static_cast<int>(currFiltered.size()) < min_tracking_pts_)
        {
            // Tracking lost — signal ORB to reactivate, clear points
            orb_active_->store(true);
            prev_pts_.clear();
        }
        else
        {
            // Update tracked points for next frame
            prev_pts_ = currFiltered;

            if (!currFiltered.empty())
            {
                cv::Point2f center(0, 0);
                for (const auto& pt : currFiltered) center += pt;
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
    size_t                   frame_idx_        = 0;
    int                      min_tracking_pts_;
    int                      win_size_;
    int                      pyr_levels_;
    bool                     reset_trajectory_next_ = false;
};
