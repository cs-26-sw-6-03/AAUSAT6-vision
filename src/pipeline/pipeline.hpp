#pragma once

/* 
 * pipeline.hpp
 *
 * Defines the Pipeline class. Owns all stages and the router.
 * Responsible for wiring stages to the router, lifecycle management, and injecting the first frame into the pipeline.
 */

#include "threadedstage.hpp"
#include "router.hpp"
#include "../utils/Config.hpp"

#include <unordered_map>
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

// Usage:
//   Pipeline pipeline(config);
//   pipeline.add_stage(std::make_shared<CaptureStage>(...));
//   pipeline.add_stage(std::make_shared<OpticalFlowStage>(...));
//   ...
//   pipeline.start();
//   pipeline.push(ctx);   // Inject frames (or let CaptureStage do it internally)
//   pipeline.stop();
class Pipeline {
public:
    explicit Pipeline(const Config& config);

    // Register a stage. Stages are started in insertion order.
    // The stage's queue is registered with the Router under stage->name().
    void add_stage(std::shared_ptr<ThreadedStage> stage);

    // Start all stages
    void start();

    // Stop all stages (blocks until all threads have joined)
    void stop();

    // Inject a frame directly — bypasses CaptureStage, useful for testing
    // or when capture is handled externally.
    // Routes to the first registered stage by default, or to a named stage.
    void push(std::shared_ptr<FrameContext> ctx,
              const std::string& target_stage = "");

    std::shared_ptr<Router> router() { return router_; }

    bool is_running() const { return running_; }

private:
    std::shared_ptr<Router>                          router_;
    std::vector<std::shared_ptr<ThreadedStage>>      stage_order_;
    std::unordered_map<std::string,
                       std::shared_ptr<ThreadedStage>> stages_;
    bool running_ = false;
    const Config& config_;
};