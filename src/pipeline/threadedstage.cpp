/* 
 * threadedstage.hpp
 *
 * Implementation of threadsafe stages
 */

#include "threadedstage.hpp"
#include <stdexcept>
#include <iostream>

ThreadedStage::ThreadedStage(std::string name,
                             std::shared_ptr<Router> router,
                             size_t queue_size)
    : Stage(std::move(name))
    , router_(std::move(router))
    , queue_(std::make_shared<FrameQueue>(queue_size))
{}
 
ThreadedStage::~ThreadedStage() {
    stop();
}
 
void ThreadedStage::start() {
    if (running_.exchange(true)) return;
    init();
    thread_ = std::thread(&ThreadedStage::run, this);
}
 
void ThreadedStage::stop() {
    if (!running_.exchange(false)) return;
    queue_->stop();
    if (thread_.joinable()) thread_.join();
    shutdown();
}
 
void ThreadedStage::enqueue(std::shared_ptr<FrameContext> ctx) {
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
 
        try {
            process(ctx);
        } catch (const std::exception& e) {
            std::cerr << "[" << name() << "] process() threw: " << e.what() << "\n";
            ctx->flags.drop_frame = true;
        }
 
        router_->dispatch(std::move(ctx));
    }
}