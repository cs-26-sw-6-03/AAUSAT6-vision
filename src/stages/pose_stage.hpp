#pragma once

#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

class PoseStage : public ThreadedStage
{
public:
    PoseStage(shared_ptr<Router> router, const Config &cfg, int cpu_affinity = -1)
        : ThreadedStage("pose", std::move(router), cfg.get<int>("pose.queue_size", 1), cpu_affinity)
        , MIN_GOOD_MATCHES_(cfg.get<int>("pose.MIN_GOOD_MATCHES", 8))
        , ALPHA_(cfg.get<float>("pose.ALPHA", 0.4f)) // The exponential moving averages (weight) for findineg smooth center of detected area
        , crop_w_(cfg.get<int>("pose.crop_width", 0))
        , crop_h_(cfg.get<int>("pose.crop_height", 0))
    {
    }

    void init() override
    {
        smoothed_initialized_ = false;
    }

    void process(shared_ptr<FrameContext> ctx) override
    {
        // Always emplace pose_result and set has_pose so the frame continues to output
        auto& pr = ctx->pose_result.emplace();
        pr.valid  = false;
        ctx->flags.has_pose        = true;
        ctx->flags.skip_processing = false;

        bool has_active_detection =
            ctx->orb_result.has_value() &&
            ctx->matching_result.has_value() &&
            (int)ctx->matching_result->matches.size() >= MIN_GOOD_MATCHES_;

        if (!has_active_detection)
        {
            // Propagate smoothedCenter (raw coords) using optical flow mean displacement.
            // This keeps the crop tracking the object as the camera pans.
            if (smoothed_initialized_ && ctx->optical_flow_result.has_value())
                propagateCenter(ctx->optical_flow_result.value());

            Point2f center = smoothed_initialized_
                ? stabilizedCenter(ctx)
                : Point2f(ctx->frame.cols / 2.f, ctx->frame.rows / 2.f);
            crop(ctx, center);
            return;
        }

        vector<Point2f> ptsFrame, ptsObject;
        for (const auto &m : ctx->matching_result->matches)
        {
            ptsFrame.push_back(ctx->orb_result->keypoints[m.queryIdx].pt);
            ptsObject.push_back(ctx->orb_result->object_keypoints[m.trainIdx].pt);
        }

        // ORB keypoints were detected in the pre-stabilization frame.
        // Transform them into the stabilized frame's coordinate space.
        if (ctx->ransac_result.has_value() && !ctx->ransac_result->homography.empty())
        {
            vector<Point2f> warped;
            cv::perspectiveTransform(ptsFrame, warped, ctx->ransac_result->homography);
            ptsFrame = warped;
        }

        Mat inlierMask;
        Mat H = findHomography(ptsObject, ptsFrame, RANSAC, 3.0, inlierMask);

        if (H.empty()) {
            crop(ctx, smoothed_initialized_ ? stabilizedCenter(ctx)
                                            : Point2f(ctx->frame.cols / 2.f, ctx->frame.rows / 2.f));
            return;
        }

        int inlierCount = countNonZero(inlierMask);
        if (inlierCount < 4) {
            std::cerr << "[PoseStage] too few inliers (" << inlierCount << ") — using last known center\n";
            crop(ctx, smoothed_initialized_ ? stabilizedCenter(ctx)
                                            : Point2f(ctx->frame.cols / 2.f, ctx->frame.rows / 2.f));
            return;
        }

        // Compute detected center as centroid of INLIER frame keypoints.
        // This is always within the frame bounds and avoids projecting the
        // reference-image center through a potentially noisy homography
        // (which can land outside the frame if the reference image is large).
        Point2f detectedCenter(0, 0);
        for (int i = 0; i < (int)ptsFrame.size(); ++i)
            if (inlierMask.at<uchar>(i))
                detectedCenter += ptsFrame[i];
        detectedCenter *= (1.0f / static_cast<float>(inlierCount));

        std::cerr << "[PoseStage] detection: center=(" << detectedCenter.x << ","
                  << detectedCenter.y << ") inliers=" << inlierCount
                  << "/" << ctx->matching_result->matches.size() << "\n";

        // detectedCenter is in stabilized frame coordinates (ptsFrame was warped above).
        // Un-warp to raw frame coordinates so that smoothedCenter is always in raw space
        // (consistent with optical-flow propagation).
        Point2f detectedRaw = detectedCenter;
        if (ctx->ransac_result.has_value() && !ctx->ransac_result->homography.empty())
        {
            vector<Point2f> pts = {detectedCenter}, raw;
            cv::perspectiveTransform(pts, raw, ctx->ransac_result->homography.inv());
            detectedRaw = raw[0];
        }

        // EMA smoothing in raw coordinates.
        if (!smoothed_initialized_) {
            smoothedCenter        = detectedRaw;
            smoothed_initialized_ = true;
        } else {
            smoothedCenter = ALPHA_ * detectedRaw + (1.f - ALPHA_) * smoothedCenter;
        }

        Point2f cropCenter = stabilizedCenter(ctx);
        pr.center     = cropCenter;
        pr.valid      = true;
        pr.confidence = (float)inlierCount / (float)ctx->matching_result->matches.size();
        crop(ctx, cropCenter);
    }

private:
    // Propagate smoothedCenter (raw coords) by the mean optical flow displacement.
    // Called every frame when ORB is passive so the crop follows scene motion.
    void propagateCenter(const OpticalFlowResult& of)
    {
        if (of.points_prev.empty() || of.points_curr.empty()) return;
        cv::Point2f sum(0, 0);
        for (size_t i = 0; i < of.points_curr.size(); ++i)
            sum += of.points_curr[i] - of.points_prev[i];
        smoothedCenter += sum * (1.0f / static_cast<float>(of.points_curr.size()));
    }

    // Transform smoothedCenter (raw coords) to stabilized frame coordinates.
    // If no correction warp exists (EdRansac skipped), raw == stabilized.
    Point2f stabilizedCenter(const shared_ptr<FrameContext>& ctx) const
    {
        if (!ctx->ransac_result.has_value() || ctx->ransac_result->homography.empty())
            return smoothedCenter;
        vector<Point2f> pts = {smoothedCenter}, out;
        cv::perspectiveTransform(pts, out, ctx->ransac_result->homography);
        return out[0];
    }

    Rect compute_roi(Point2f center, int src_w, int src_h) const
    {
        int half_w = crop_w_ / 2;
        int half_h = crop_h_ / 2;

        int x = static_cast<int>(center.x) - half_w;
        int y = static_cast<int>(center.y) - half_h;

        x = std::max(0, std::min(x, src_w - crop_w_));
        y = std::max(0, std::min(y, src_h - crop_h_));

        return { x, y, crop_w_, crop_h_ };
    }

    void crop(shared_ptr<FrameContext> ctx, Point2f center)
    {
        if (crop_w_ <= 0 || crop_h_ <= 0 ||
            crop_w_ >= ctx->frame.cols || crop_h_ >= ctx->frame.rows)
            return;

        Rect roi = compute_roi(center, ctx->frame.cols, ctx->frame.rows);
        ctx->frame = ctx->frame(roi).clone();
    }

    int     MIN_GOOD_MATCHES_;
    float   ALPHA_;
    int     crop_w_;
    int     crop_h_;
    Point2f smoothedCenter;           // always in RAW (pre-warp) frame coordinates
    bool    smoothed_initialized_ = false;
};
