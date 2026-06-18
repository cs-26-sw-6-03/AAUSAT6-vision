#pragma once

#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include <atomic>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>
#include <array>

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

        // Reserve once; avoids repeated heap allocations across frames
        prev_pts_.reserve(512);
        curr_pts_buf_.reserve(512);
        status_buf_.reserve(512);
        err_buf_.reserve(512);
        prev_filtered_buf_.reserve(512);
        curr_filtered_buf_.reserve(512);

        accumulated_params_.clear();
        accumulated_params_.reserve(window_frames_);

        average_transform_  = cv::Mat::eye(3, 3, CV_64F);
        accumulated_count_  = 0;
        propagation_count_  = 0;
        propagation_center_ = cv::Point2f(-1.f, -1.f);
        orb_active_->store(true, std::memory_order_relaxed);
    }

    void process(std::shared_ptr<FrameContext> ctx) override {
        cv::cvtColor(ctx->frame, gray_buf_, cv::COLOR_BGR2GRAY);

        bool fresh_detection = ctx->orb_result.has_value() &&
                               ctx->orb_result->has_matches &&
                               ctx->matching_result.has_value() &&
                               !ctx->matching_result->matches.empty();

        if (fresh_detection) {
            const auto &kps     = ctx->orb_result->keypoints;
            const auto &matches = ctx->matching_result->matches;

            prev_pts_.clear();
            prev_pts_.reserve(matches.size());
            for (const auto &m : matches)
                prev_pts_.push_back(kps[m.queryIdx].pt);

            // Swap avoids a deep copy — gray_buf_ is still valid after this
            cv::swap(prev_gray_, gray_buf_);

            phase_ = Phase::ACCUMULATING;
            accumulated_params_.clear();
            accumulated_count_  = 0;
            propagation_center_ = cv::Point2f(-1.f, -1.f);
            orb_active_->store(false, std::memory_order_relaxed);

            apply_identity(ctx);
            return;
        }

        switch (phase_) {
        case Phase::IDLE:
            apply_identity(ctx);
            return;
        case Phase::ACCUMULATING:
            do_accumulate(ctx);
            return;
        case Phase::PROPAGATING:
            do_propagate(ctx);
            return;
        }
    }

  private:
    // Compact affine storage: [a, b, tx, c, d, ty] — 6 doubles instead of a 3x3 Mat
    using AffineParams = std::array<double, 6>;

    enum class Phase { IDLE, ACCUMULATING, PROPAGATING };

    std::shared_ptr<std::atomic<bool>> orb_active_;

    int win_size_;
    int pyr_levels_;
    int min_tracking_pts_;
    int max_propagation_frames_;
    int window_frames_;

    Phase phase_ = Phase::IDLE;

    // Persistent buffers — allocated once in init(), reused every frame
    cv::Mat                  gray_buf_;
    cv::Mat                  prev_gray_;
    std::vector<cv::Point2f> prev_pts_;
    std::vector<cv::Point2f> curr_pts_buf_;
    std::vector<uchar>       status_buf_;
    std::vector<float>       err_buf_;
    std::vector<cv::Point2f> prev_filtered_buf_;
    std::vector<cv::Point2f> curr_filtered_buf_;

    // Stores compact affine params rather than full 3x3 Mats
    std::vector<AffineParams> accumulated_params_;
    int                       accumulated_count_ = 0;

    cv::Mat     average_transform_  = cv::Mat::eye(3, 3, CV_64F);
    int         propagation_count_  = 0;
    cv::Point2f propagation_center_ = cv::Point2f(-1.f, -1.f);

    // Compute centroid without a separate loop — fused single pass
    static cv::Point2f centroid(const std::vector<cv::Point2f> &pts) {
        cv::Point2f c(0.f, 0.f);
        for (const auto &p : pts)
            c += p;
        return c * (1.f / static_cast<float>(pts.size()));
    }

    // Build a 3x3 homogeneous matrix from a 2x3 affine Mat
    static AffineParams mat_to_params(const cv::Mat &M23) {
        // M23 is CV_64F, 2 rows x 3 cols
        const double *r0 = M23.ptr<double>(0);
        const double *r1 = M23.ptr<double>(1);
        return { r0[0], r0[1], r0[2],
                 r1[0], r1[1], r1[2] };
    }


    void do_accumulate(std::shared_ptr<FrameContext> ctx) {
        if (prev_pts_.empty() || prev_gray_.empty()) {
            apply_identity(ctx);
            return;
        }

        // Reuse persistent buffers — no heap allocation per frame
        cv::calcOpticalFlowPyrLK(prev_gray_, gray_buf_, prev_pts_, curr_pts_buf_,
                                 status_buf_, err_buf_,
                                 cv::Size(win_size_, win_size_), pyr_levels_,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), 0);

        prev_filtered_buf_.clear();
        curr_filtered_buf_.clear();
        const size_t n = status_buf_.size();
        for (size_t i = 0; i < n; ++i) {
            if (status_buf_[i]) {
                prev_filtered_buf_.push_back(prev_pts_[i]);
                curr_filtered_buf_.push_back(curr_pts_buf_[i]);
            }
        }

        if (static_cast<int>(curr_filtered_buf_.size()) < min_tracking_pts_) {
            orb_active_->store(true, std::memory_order_relaxed);
            phase_ = Phase::IDLE;
            prev_pts_.clear();
            apply_identity(ctx);
            return;
        }

        cv::Point2f prev_c = centroid(prev_filtered_buf_);
        cv::Point2f curr_c = centroid(curr_filtered_buf_);
        AffineParams fwd_params = { 1.0, 0.0, static_cast<double>(curr_c.x - prev_c.x),
                                    0.0, 1.0, static_cast<double>(curr_c.y - prev_c.y) };

        accumulated_params_.push_back(fwd_params);
        ++accumulated_count_;

        ctx->ransac_result.emplace();
        ctx->ransac_result->homography = cv::Mat::eye(3, 3, CV_32F);

        auto &ofr        = ctx->optical_flow_result.emplace();
        ofr.points_prev  = prev_filtered_buf_;
        ofr.points_curr  = curr_filtered_buf_;
        ofr.suggested_center = centroid(curr_filtered_buf_);

        // Swap — O(1), avoids clone()
        prev_pts_.swap(curr_filtered_buf_);
        cv::swap(prev_gray_, gray_buf_);

        if (accumulated_count_ >= window_frames_) {
            // Accumulate average directly from compact params — no Mat overhead
            AffineParams avg = {};
            for (const auto &ap : accumulated_params_)
                for (int k = 0; k < 6; ++k)
                    avg[k] += ap[k];
            const double inv_n = 1.0 / static_cast<double>(accumulated_params_.size());
            for (double &v : avg) v *= inv_n;

            // Write average into the 3x3 Mat used by perspectiveTransform in propagation
            double *r0 = average_transform_.ptr<double>(0);
            double *r1 = average_transform_.ptr<double>(1);
            double *r2 = average_transform_.ptr<double>(2);
            r0[0]=avg[0]; r0[1]=avg[1]; r0[2]=avg[2];
            r1[0]=avg[3]; r1[1]=avg[4]; r1[2]=avg[5];
            r2[0]=0.0;    r2[1]=0.0;    r2[2]=1.0;

            propagation_center_ = prev_pts_.empty()
                                      ? cv::Point2f(-1.f, -1.f)
                                      : centroid(prev_pts_);
            prev_pts_.clear();
            prev_gray_.release();

            phase_             = Phase::PROPAGATING;
            propagation_count_ = 0;
        }

        ctx->flags.has_inliers = true;
    }

    void do_propagate(std::shared_ptr<FrameContext> ctx) {
        if (++propagation_count_ > max_propagation_frames_) {
            reset_to_idle(ctx);
            return;
        }

        if (propagation_center_.x < 0.f ||
            propagation_center_.x >= static_cast<float>(ctx->frame.cols) ||
            propagation_center_.y < 0.f ||
            propagation_center_.y >= static_cast<float>(ctx->frame.rows)) {
            reset_to_idle(ctx);
            return;
        }

        // Inline the 3x3 perspective transform for a single point —
        // avoids vector allocation and the function-call overhead of perspectiveTransform
        const double *r0 = average_transform_.ptr<double>(0);
        const double *r1 = average_transform_.ptr<double>(1);
        const double *r2 = average_transform_.ptr<double>(2);
        const double  x  = propagation_center_.x;
        const double  y  = propagation_center_.y;
        const double  w  = r2[0]*x + r2[1]*y + r2[2];
        const double inv_w = 1.0 / w;
        const cv::Point2f estimated(
            static_cast<float>((r0[0]*x + r0[1]*y + r0[2]) * inv_w),
            static_cast<float>((r1[0]*x + r1[1]*y + r1[2]) * inv_w));

        auto &ofr            = ctx->optical_flow_result.emplace();
        ofr.points_prev      = { propagation_center_ };
        ofr.points_curr      = { estimated };
        ofr.suggested_center = estimated;

        propagation_center_            = estimated;
        ctx->ransac_result.emplace();
        ctx->ransac_result->homography = cv::Mat::eye(3, 3, CV_32F);
        ctx->flags.has_inliers         = true;
    }

    void reset_to_idle(std::shared_ptr<FrameContext> ctx) {
        orb_active_->store(true, std::memory_order_relaxed);
        phase_              = Phase::IDLE;
        propagation_center_ = cv::Point2f(-1.f, -1.f);
        apply_identity(ctx);
    }

    void apply_identity(std::shared_ptr<FrameContext> ctx) {
        ctx->ransac_result.emplace();
        ctx->ransac_result->homography = cv::Mat::eye(3, 3, CV_32F);
        ctx->flags.has_inliers         = true;
    }
};