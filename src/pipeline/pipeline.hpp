#pragma once

#include "../utils/config.hpp"
#include "router.hpp"
#include "threadedstage.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

class Pipeline {
  public:
    explicit Pipeline(const Config &config);

    void add_stage(std::shared_ptr<ThreadedStage> stage);

    void start();

    void stop();

    void push(std::shared_ptr<FrameContext> ctx,
              const std::string            &target_stage = "");

    std::shared_ptr<Router> router() { return router_; }

    bool is_running() const { return running_; }

  private:
    std::shared_ptr<Router>                     router_;
    std::vector<std::shared_ptr<ThreadedStage>> stage_order_;
    std::unordered_map<std::string,
                       std::shared_ptr<ThreadedStage>>
                  stages_;
    bool          running_ = false;
    const Config &config_;
};