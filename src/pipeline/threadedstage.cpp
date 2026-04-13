/* 
 * threadedstage.hpp
 *
 * Implementation of threadsafe stages
 */

#include "threadedstage.hpp"
#include <stdexcept>
#include <iostream>
#include <chrono>

ThreadedStage::ThreadedStage(std::string name,
                             std::shared_ptr<Router> router,
                             size_t queue_size,
                             int cpu_affinity)
    : Stage(std::move(name))
    , router_(std::move(router))
    , queue_(std::make_shared<FrameQueue>(queue_size))
    , cpu_affinity_(cpu_affinity)
{}
 
ThreadedStage::~ThreadedStage() {
    stop();
}
 
void ThreadedStage::start() {
    if (running_.exchange(true)) return;
    init();
    thread_ = std::thread(&ThreadedStage::run, this);

          if (cpu_affinity_ >= 0) {
          cpu_set_t cpuset;
          CPU_ZERO(&cpuset);
          CPU_SET(cpu_affinity_, &cpuset);
          pthread_setaffinity_np(thread_.native_handle(), sizeof(cpu_set_t), &cpuset);
      }
}
 
void ThreadedStage::stop() {
    if (!running_.exchange(false)) return;
    queue_->stop();
    if (thread_.joinable()) thread_.join();
    shutdown();
}
 
void ThreadedStage::enqueue(std::shared_ptr<FrameContext> ctx) {
    ctx->telemetry.per_stage[name()].queue_enter = std::chrono::steady_clock::now();
    queue_->push(std::move(ctx));
}
 
void ThreadedStage::dispatch(std::shared_ptr<FrameContext> ctx) {
    router_->dispatch(std::move(ctx));
}
 
// Default run loop — pops from queue, calls process(), dispatches result.
// Source stages (e.g. CaptureStage) override this entirely.
void ThreadedStage::run() {
    while (running_.load()) {
        auto maybe_ctx = queue_->pop_for(POP_TIMEOUT);
        if (!maybe_ctx.has_value()) continue;
 
        auto ctx = std::move(maybe_ctx.value());
        if (!ctx) continue;

        auto &timing = ctx->telemetry.per_stage[name()];
        timing.queue_dequeue = std::chrono::steady_clock::now();
        try {
            timing.process_start = std::chrono::steady_clock::now();
            process(ctx);
            timing.process_end = std::chrono::steady_clock::now();
        } catch (const std::exception& e) {
            timing.process_end = std::chrono::steady_clock::now();
            std::cerr << "[" << name() << "] process() threw: " << e.what() << "\n";
            ctx->flags.drop_frame = true;
        }
 
        router_->dispatch(std::move(ctx));
    }
}