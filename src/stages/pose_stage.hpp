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
    PoseStage(shared_ptr<Router> router, const Config &cfg)
        : ThreadedStage("pose", std::move(router), 1)
        , MIN_GOOD_MATCHES_(cfg.get<int>("MIN_GOOD_MATCHES", 8))
        , ALPHA_(cfg.get<float>("ALPHA", 0.4f))
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
        // Always emplace pose_result and set has_pose so the frame continues to ransac
        auto& pr = ctx->pose_result.emplace();
        pr.valid  = false;
        ctx->flags.has_pose        = true;
        ctx->flags.skip_processing = false;

        // No ORB result or no matches (ORB was passive, or active but no DB match)
        if (!ctx->orb_result.has_value() ||
            !ctx->matching_result.has_value() ||
            (int)ctx->matching_result->matches.size() < MIN_GOOD_MATCHES_)
        {
            // Crop to frame center as fallback
            Point2f center(ctx->frame.cols / 2.f, ctx->frame.rows / 2.f);
            crop(ctx, center);
            return;
        }

        vector<Point2f> ptsFrame, ptsObject;
        for (const auto &m : ctx->matching_result->matches)
        {
            ptsFrame.push_back(ctx->orb_result->keypoints[m.queryIdx].pt);
            ptsObject.push_back(ctx->orb_result->object_keypoints[m.trainIdx].pt);
        }

        Mat inlierMask;
        Mat H = findHomography(ptsObject, ptsFrame, RANSAC, 3.0, inlierMask);

        if (H.empty()) {
            Point2f center(ctx->frame.cols / 2.f, ctx->frame.rows / 2.f);
            crop(ctx, center);
            return;
        }

        int inlierCount = countNonZero(inlierMask);
        if (inlierCount < MIN_GOOD_MATCHES_) {
            Point2f center(ctx->frame.cols / 2.f, ctx->frame.rows / 2.f);
            crop(ctx, center);
            return;
        }

        // Project the reference image's center through H to find where the object appears in the frame.
        // H maps object-space → frame-space, so we must use the reference image dimensions here,
        // not the current frame dimensions.
        cv::Size objSize = ctx->orb_result->object_size;
        Point2f refCenter(objSize.width / 2.f, objSize.height / 2.f);
        vector<Point2f> refPts = {refCenter};
        vector<Point2f> projectedPts;
        perspectiveTransform(refPts, projectedPts, H);

        Point2f detectedCenter = projectedPts[0];
        if (detectedCenter.x < 0 || detectedCenter.y < 0 ||
            detectedCenter.x >= ctx->frame.cols ||
            detectedCenter.y >= ctx->frame.rows)
        {
            Point2f center(ctx->frame.cols / 2.f, ctx->frame.rows / 2.f);
            crop(ctx, center);
            return;
        }

        // Smooth the detection center (EMA). Initialize on first valid detection.
        if (!smoothed_initialized_) {
            smoothedCenter        = detectedCenter;
            smoothed_initialized_ = true;
        } else {
            smoothedCenter = ALPHA_ * detectedCenter + (1.f - ALPHA_) * smoothedCenter;
        }

        pr.center     = smoothedCenter;
        pr.valid      = true;
        pr.confidence = (float)inlierCount / (float)ctx->matching_result->matches.size();
        crop(ctx, smoothedCenter);
    }

private:
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
        // Skip if crop dimensions not configured or source is already smaller
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
    Point2f smoothedCenter;
    bool    smoothed_initialized_ = false;
};
