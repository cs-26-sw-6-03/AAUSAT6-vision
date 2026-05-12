#pragma once
#include "../utils/threadsafequeue.hpp"
#include "framecontext.hpp"
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

using FrameQueue = ThreadSafeQueue<std::shared_ptr<FrameContext>>;

class Router {
  public:

    void register_stage(const std::string &name, std::shared_ptr<FrameQueue> queue);

    bool dispatch(std::shared_ptr<FrameContext> ctx);

    void set_routing_fn(std::function<std::string(const FrameContext &)> fn);

  private:
    static std::string default_route(const FrameContext &ctx);

    std::unordered_map<std::string, std::shared_ptr<FrameQueue>> queues_;
    std::function<std::string(const FrameContext &)>             routing_fn_;
};
