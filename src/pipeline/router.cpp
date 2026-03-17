/* 
 * router.cpp
 *
 * Implementation of the Router class.
 */

#include "router.hpp"
#include <stdexcept>

void Router::register_stage(const std::string& name, std::shared_ptr<FrameQueue> queue) {
    queues_[name] = std::move(queue);
}
 
void Router::set_routing_fn(std::function<std::string(const FrameContext&)> fn) {
    routing_fn_ = std::move(fn);
}
 
bool Router::dispatch(std::shared_ptr<FrameContext> ctx) {
    if (!ctx) return false;
    if (ctx->flags.drop_frame) return false;
 
    std::string target;
    if (routing_fn_) {
        target = routing_fn_(*ctx);
    } else {
        target = default_route(*ctx);
    }
 
    if (target.empty()) return false;
 
    auto it = queues_.find(target);
    if (it == queues_.end()) {
        throw std::runtime_error("Router: no queue registered for stage '" + target + "'");
    }
 
    it->second->push(std::move(ctx));
    return true;
}
 
// Default routing table. Reads RoutingFlags in order of pipeline progression.
// Stage progression:
//   capture -> optical_flow -> [orb | output]
//                orb -> matching -> ransac -> pose -> output
std::string Router::default_route(const FrameContext& ctx) const {
    const auto& f = ctx.flags;
 
    if (f.drop_frame)       return "";
    if (f.skip_processing)  return "output";
    if (f.has_pose)         return "output";
    if (f.has_inliers)      return "pose";
    if (f.has_matches)      return "ransac";
    if (f.has_keypoints)    return "matching";
    if (f.from_input)       return "optical_flow";
    if (f.needs_redetect)   return "orb";

 
    return "";
}