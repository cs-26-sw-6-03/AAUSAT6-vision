#pragma once

/* 
 * threadedstage.hpp
 *
 * A stage that runs on its own thread. Frames are enqueued in hte input queue by the router.
 * Afterwhich the frames are processed and dispatched back through the router.
 */

#include "stage.hpp"
#include "router.hpp"
#include "../utils/threadsafequeue.hpp"

#include <thread>
#include <atomic>
#include <memory>
#include <chrono>

// Two modes:
//   Queue mode (default) — for processing stages.
//     The Router pushes frames into the stage's input queue.
//     The thread pops frames, calls process(), then dispatches via Router.
//   Source mode — for capture/source stages that generate frames themselves.
//     Pass is_source = true in the constructor.
//     The thread calls run_source() in a loop instead of popping from a queue.
//     Subclass calls dispatch() manually to push produced frames into the pipeline.
//
// Lifecycle (both modes):
//   start() -> [thread runs] -> stop()
class ThreadedStage : public Stage {
public:
    ThreadedStage(std::string name,
                  std::shared_ptr<Router> router,
                  size_t queue_size = 32);
 
    ~ThreadedStage() override;
 
    void start();
    void stop();
 
    void enqueue(std::shared_ptr<FrameContext> ctx);
    std::shared_ptr<FrameQueue> queue() { return queue_; }
    std::shared_ptr<Router> router() { return router_; }
    bool is_running() const { return running_.load(); }
 
protected:
    // Queue mode: subclasses implement this
    void process(std::shared_ptr<FrameContext> ctx) override {}
 
    // Source mode: override run() entirely to produce frames instead of consuming queue.
    // Call router()->dispatch(ctx) to push frames into the pipeline.
    virtual void run();
 
    // Convenience wrapper used by source stages
    void dispatch(std::shared_ptr<FrameContext> ctx);
 
private:
    std::shared_ptr<Router>     router_;
    std::shared_ptr<FrameQueue> queue_;
    std::thread                 thread_;
    std::atomic<bool>           running_{false};
 
    static constexpr std::chrono::milliseconds POP_TIMEOUT{100};
};