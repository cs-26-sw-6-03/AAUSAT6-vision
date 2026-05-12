#pragma once
#include "framecontext.hpp"
#include <memory>
#include <string>

class Stage {
  public:
    explicit Stage(std::string name)
        : name_(std::move(name)) {}
    virtual ~Stage() = default;

    virtual void process(std::shared_ptr<FrameContext> ctx) = 0;

    //Called before pipeline starts
    virtual void init() {}

    // Called once after pipeline stop
    virtual void shutdown() {}

    const std::string &name() const { return name_; }

  private:
    std::string name_;
};