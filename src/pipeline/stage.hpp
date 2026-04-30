#pragma once

/*
 * stage.hpp
 *
 * Abstract base class for pipeline stages. Each stage is responsible for processing a FrameContext and setting routing flags to indicate where the frame should go next.
 */

#include "framecontext.hpp"
#include <memory>
#include <string>

class Stage {
  public:
    explicit Stage(std::string name)
        : name_(std::move(name)) {}
    virtual ~Stage() = default;

    // Process a single frame. Implementations should:
    //   1. Read what they need from ctx
    //   2. Write results back into ctx
    //   3. Set the appropriate RoutingFlags
    virtual void process(std::shared_ptr<FrameContext> ctx) = 0;

    // Called once before the pipeline starts. Load models, open devices, etc.
    virtual void init() {}

    // Called once after the pipeline stops. Release resources.
    virtual void shutdown() {}

    const std::string &name() const { return name_; }

  private:
    std::string name_;
};