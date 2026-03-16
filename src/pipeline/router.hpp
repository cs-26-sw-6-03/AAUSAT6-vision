#pragma once

/* 
 * router.hpp
 *
 * Defines the Router class, which manages the routing of FrameContext objects
 * between different stages of the vision pipeline. Each stage registers itself
 * with the Router, providing a name and an input queue. The Router then dispatches
 * frames to the appropriate stage based on flags set in the FrameContext.
 */

#include "framecontext.hpp"
#include "../utils/threadsafequeue.hpp"
#include <memory>
#include <unordered_map>
#include <functional>
#include <string>

using FrameQueue = ThreadSafeQueue<std::shared_ptr<FrameContext>>;

class Router {
public:
    // Register a named stage and its input queue
    void register_stage(const std::string& name, std::shared_ptr<FrameQueue> queue);

    // Core dispatch — reads FrameContext flags and pushes to the correct queue.
    // Returns false if no route matched.
    bool dispatch(std::shared_ptr<FrameContext> ctx);

    // Override the routing logic with a custom function.
    // Signature: (const FrameContext&) -> std::string  (stage name to route to)
    // Useful for experiments where you want to hardwire a specific path.
    void set_routing_fn(std::function<std::string(const FrameContext&)> fn);

private:
    std::string default_route(const FrameContext& ctx) const;

    std::unordered_map<std::string, std::shared_ptr<FrameQueue>> queues_;
    std::function<std::string(const FrameContext&)>              routing_fn_;
};
