#pragma once

/*
 * pose_stage.hpp
 *
 * Stub pose estimation stage. Receives frames after optical flow and before
 * RANSAC stabilization. Currently passes frames through and sets has_pose=true.
 */

#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"

class PoseStage : public ThreadedStage {
public:
    PoseStage(std::shared_ptr<Router> router, const Config& cfg)
        : ThreadedStage("pose", std::move(router), cfg.get<int>("pipeline.queue_size", 32))
    {}

    void process(std::shared_ptr<FrameContext> ctx) override {
        ctx->flags.skip_processing = false;
        ctx->flags.has_pose        = true;
    }
};
