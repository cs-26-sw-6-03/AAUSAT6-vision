#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

class EdRansacStage : public ThreadedStage
{
public:
    EdRansacStage(std::shared_ptr<Router> router, const Config &cfg, int cpu_affinity = -1)
        : ThreadedStage("ransac", std::move(router), cfg.get<int>("pipeline.queue_size", 32), cpu_affinity)
    {
        lowe_ratio           = cfg.get<float>("stabilizer.lowe_ratio", 0.75f);
        ransac_reproj_thresh = cfg.get<float>("stabilizer.reprojection_threshold", 3.0);
        ed_threshold         = cfg.get<float>("stabilizer.ed_threshold", 0.5f);
        min_inliers          = cfg.get<int>("stabilizer.min_inlies", 10);
        smooth_radius        = cfg.get<int>("stabilizer.smooth_radius", 15);
    }

    void init() override
    {
        matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, false);
        trajectory_.clear();
        prev_kps_.clear();
        prev_desc_.release();
        frame_idx_   = 0;
        initialized_ = false;
    }

    void process(std::shared_ptr<FrameContext> ctx) override
    {
        std::vector<cv::Point2f> pts_prev, pts_curr;

        if (ctx->orb_result.has_value() && !ctx->orb_result->descriptors.empty())
        {
            const auto& curr_kps  = ctx->orb_result->keypoints;
            const auto& curr_desc = ctx->orb_result->descriptors;

            if (!initialized_ || prev_desc_.empty()) {
                // First ORB frame: seed buffer, push identity, skip stabilization
                trajectory_.push_back(cv::Mat::eye(3, 3, CV_64F));
                prev_kps_    = curr_kps;
                curr_desc.copyTo(prev_desc_);
                initialized_ = true;
                ++frame_idx_;
                ctx->flags.has_pose    = false;
                ctx->flags.has_inliers = true;
                return;
            }

            if (ctx->optical_flow_result->tracking_just_seeded) {
                // Seeding frame: reset trajectory, update buffer, skip warp
                trajectory_.clear();
                trajectory_.push_back(cv::Mat::eye(3, 3, CV_64F));
                prev_kps_ = curr_kps;
                curr_desc.copyTo(prev_desc_);
                ctx->flags.has_inliers = true;
                ++frame_idx_;
                return;
            }

            // LK is preferred when available — it spans exactly one frame.
            // Only use ORB adjacent-frame matching as fallback when LK has no result
            // (e.g. immediately after a tracking loss before LK has caught up).
            bool lk_available = ctx->optical_flow_result.has_value() &&
                                 !ctx->optical_flow_result->points_prev.empty();
            if (!lk_available) {
                std::vector<std::vector<cv::DMatch>> knn_matches;
                matcher_->knnMatch(prev_desc_, curr_desc, knn_matches, 2);
                for (const auto& m : knn_matches) {
                    if (m.size() < 2) continue;
                    if (m[0].distance < lowe_ratio * m[1].distance) {
                        pts_prev.push_back(prev_kps_[m[0].queryIdx].pt);
                        pts_curr.push_back(curr_kps[m[0].trainIdx].pt);
                    }
                }
            }

            // Always update ORB buffer so future ORB-only frames have fresh descriptors
            prev_kps_ = curr_kps;
            cv::swap(prev_desc_, ctx->orb_result->descriptors);
        }

        // Prefer LK correspondences (single-frame motion, reliable at any speed)
        if (pts_prev.empty() &&
            ctx->optical_flow_result.has_value() &&
            !ctx->optical_flow_result->points_prev.empty())
        {
            pts_prev = ctx->optical_flow_result->points_prev;
            pts_curr = ctx->optical_flow_result->points_curr;
        }

        if (pts_prev.empty())
        {
            ctx->flags.has_inliers = true;
            return;
        }

        // ED-RANSAC homography estimation
        cv::Mat H_inter = cv::Mat::eye(3, 3, CV_64F);
        if ((int)pts_prev.size() >= min_inliers) {
            cv::Mat H = ed_ransac(pts_prev, pts_curr);
            if (!H.empty()) {
                H_inter = H;
            } else {
                std::cerr << "[EdRansacStage] ED-RANSAC failed at frame "
                          << frame_idx_ << " — using identity.\n";
            }
        } else {
            std::cerr << "[EdRansacStage] Too few matches ("
                      << pts_prev.size() << ") at frame "
                      << frame_idx_ << " — using identity.\n";
        }

        if (ctx->optical_flow_result->tracking_reseeded) {
            trajectory_.clear();
            trajectory_.push_back(cv::Mat::eye(3, 3, CV_64F));
        }

        // Accumulate trajectory: T[i] = H_inter(i-1→i) * T[i-1]
        cv::Mat T_curr = H_inter * trajectory_.back();
        trajectory_.push_back(T_curr.clone());
        if ((int)trajectory_.size() > smooth_radius + 1)
            trajectory_.erase(trajectory_.begin());

        // Smoothed trajectory (causal trailing-window average)
        cv::Mat T_smooth = smooth_transform();

        // Correction warp: what to apply to the raw frame
        cv::Mat warp = T_smooth * T_curr.inv();

        // Apply warp in-place
        cv::Mat stabilized;
        cv::warpPerspective(ctx->frame, stabilized, warp,
                            ctx->frame.size(),
                            cv::INTER_LINEAR,
                            cv::BORDER_REPLICATE);
        ctx->frame = stabilized;

        ctx->ransac_result.emplace();
        ctx->ransac_result->homography = warp;
        ctx->flags.has_inliers = true;
        ++frame_idx_;
    }

private:

    float  lowe_ratio            = 0.75f;
    double ransac_reproj_thresh  = 3.0;   // pixels
    float  ed_threshold          = 0.5f;  // pixels
    int    min_inliers           = 10;
    int    smooth_radius         = 15;    // trailing frames

    cv::Ptr<cv::BFMatcher> matcher_;

    // Previous-frame data (for adjacent-frame registration)
    std::vector<cv::KeyPoint> prev_kps_;
    cv::Mat                   prev_desc_;

    std::vector<cv::Mat> trajectory_;

    bool initialized_ = false;
    std::size_t frame_idx_ = 0;

    // Pass 1: standard RANSAC homography → initial inlier set
    // Pass 2: project inliers through H, discard any with ED > ed_threshold
    // Final:  least-squares re-estimation on the clean inlier set
    cv::Mat ed_ransac(const std::vector<cv::Point2f>& pts_prev,
                      const std::vector<cv::Point2f>& pts_curr) const
    {
        if ((int)pts_prev.size() < min_inliers) return {};

        // Pass 1: RANSAC
        cv::Mat inlier_mask;
        cv::Mat H = cv::findHomography(pts_prev, pts_curr,
                                       cv::RANSAC,
                                       ransac_reproj_thresh,
                                       inlier_mask);
        if (H.empty()) return {};

        std::vector<cv::Point2f> inl_prev, inl_curr;
        for (int i = 0; i < (int)pts_prev.size(); ++i) {
            if (inlier_mask.at<uchar>(i)) {
                inl_prev.push_back(pts_prev[i]);
                inl_curr.push_back(pts_curr[i]);
            }
        }
        if ((int)inl_prev.size() < min_inliers) return {};

        // Pass 2: Euclidean distance filter
        std::vector<cv::Point2f> projected;
        cv::perspectiveTransform(inl_prev, projected, H);

        std::vector<cv::Point2f> ed_prev, ed_curr;
        for (int i = 0; i < (int)inl_prev.size(); ++i) {
            float dx = projected[i].x - inl_curr[i].x;
            float dy = projected[i].y - inl_curr[i].y;
            if (std::sqrt(dx * dx + dy * dy) < ed_threshold) {
                ed_prev.push_back(inl_prev[i]);
                ed_curr.push_back(inl_curr[i]);
            }
        }
        if ((int)ed_prev.size() < min_inliers) return {};

        // Final: least-squares re-estimation on clean set
        return cv::findHomography(ed_prev, ed_curr, 0);
    }

    // Causal trailing-window average over all retained trajectory entries
    cv::Mat smooth_transform() const
    {
        if (trajectory_.empty()) return cv::Mat::eye(3, 3, CV_64F);

        cv::Mat sum = cv::Mat::zeros(3, 3, CV_64F);
        for (const auto& T : trajectory_)
            sum += T;

        return sum / (double)trajectory_.size();
    }
};