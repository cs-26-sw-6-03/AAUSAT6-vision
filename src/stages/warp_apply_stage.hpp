#pragma once

#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>


//Uses homography from RANSAC to warp the current frame and stabilize it
class WarpApplyStage : public ThreadedStage {
  public:
    WarpApplyStage(std::shared_ptr<Router> router, const Config &cfg,
                   int cpu_affinity = -1)
        : ThreadedStage("warp_apply", std::move(router), cfg.get<int>("pipeline.queue_size", 32), cpu_affinity) {
    }

    void init() override {}

    void process(std::shared_ptr<FrameContext> ctx) override {
        if (ctx->ransac_result.has_value() &&
            !ctx->ransac_result->homography.empty()) {
            //reduce the matrix to 2x3 for warpAffine
            cv::Mat warp23 = ctx->ransac_result->homography.rowRange(0, 2);

            cv::Mat stabilized;
            cv::warpAffine(ctx->frame, stabilized, warp23, ctx->frame.size(),
                           cv::INTER_NEAREST, cv::BORDER_REPLICATE);
            ctx->frame = stabilized;
        }

        ctx->flags.has_warp    = false;
        ctx->flags.has_inliers = true; // route to pose
    }
};
