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
        : ThreadedStage("capture", std::move(router), 1)
        , MIN_GOOD_MATCHES_(cfg.get<int>("MIN_GOOD_MATCHES", 8))
        , ALPHA_(cfg.get<float>("ALPHA", 0.4f))
    {
    }

    void init() override
    {
    }

    void process(shared_ptr<FrameContext> ctx) override
    {
        ctx->pose_result->valid = false;

        if (!ctx->flags.has_keypoints)
        {
            return;
        }

        if ((int)ctx->matching_result->matches.size() < MIN_GOOD_MATCHES_)
        {
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

        if (H.empty())
            return;

        int inlierCount = countNonZero(inlierMask);
        if (inlierCount < MIN_GOOD_MATCHES_)
        {
            return;
        }

        Point2f refCenter(ctx->frame.size().width / 2.f, ctx->frame.size().height / 2.f);
        vector<Point2f> refPts = {refCenter};
        vector<Point2f> projectedPts;
        perspectiveTransform(refPts, projectedPts, H);
        Mat gray_frame;
        cvtColor(ctx->frame, gray_frame, COLOR_BGR2GRAY);
        Point2f detectedCenter = projectedPts[0];
        if (detectedCenter.x < 0 || detectedCenter.y < 0 ||
            detectedCenter.x >= gray_frame.cols ||
            detectedCenter.y >= gray_frame.rows)
        {
            return;
        }

        
    }

private:
    int MIN_GOOD_MATCHES_;
    float ALPHA_;
};