#pragma once

#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// WarpApplyStage — applies the pre-computed stabilization warp to the frame.
//
// Stage 2 of the two-stage stabilizer. Runs on its own thread.
//
// Receives frames from AffineEstimatorStage (via has_warp flag) with
// ransac_result->homography already set. Applies cv::warpAffine using the
// top 2 rows of the 3x3 homography, then sets has_inliers = true to route
// the stabilized frame to PoseStage.
//
// Splitting estimation (AffineEstimatorStage) and application (this stage)
// across two threads lets frame N's warp be applied in parallel with frame
// N+1's affine estimation.
class WarpApplyStage : public ThreadedStage {
  public:
    WarpApplyStage(std::shared_ptr<Router> router, const Config &cfg,
                   int cpu_affinity = -1)
        : ThreadedStage("warp_apply", std::move(router)
        , cfg.get<int>("pipeline.queue_size", 32), cpu_affinity) {
    }

    void init() override {}

    void process(std::shared_ptr<FrameContext> ctx) override {
        if (ctx->ransac_result.has_value() &&
            !ctx->ransac_result->homography.empty()) {
            // Extract 2x3 affine from the stored 3x3 homogeneous matrix
            cv::Mat warp23 = ctx->ransac_result->homography.rowRange(0, 2);

            cv::Mat stabilized;
            cv::warpAffine(ctx->frame, stabilized, warp23, ctx->frame.size(),
                           cv::INTER_NEAREST, cv::BORDER_REPLICATE);
            ctx->frame = stabilized;
        }

        ctx->flags.has_warp = false;
        ctx->flags.has_inliers = true; // route to pose
    }
};
