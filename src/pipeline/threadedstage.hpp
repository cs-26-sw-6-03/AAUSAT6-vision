#pragma once

#include "../utils/threadsafequeue.hpp"
#include "router.hpp"
#include "stage.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <pthread.h>
#include <thread>


// can be used both as a queue and a source for frames

// queue implements the process method, source overrides run to produce frames and dispatches them directly
class ThreadedStage : public Stage {
  public:
    ThreadedStage(std::string             name,
                  std::shared_ptr<Router> router,
                  size_t                  queue_size   = 32,
                  int                     cpu_affinity = -1);

    ~ThreadedStage() override;

    void start();
    void stop();

    void                        enqueue(std::shared_ptr<FrameContext> ctx);
    std::shared_ptr<FrameQueue> queue() { return queue_; }
    std::shared_ptr<Router>     router() { return router_; }
    bool                        is_running() const { return running_.load(); }

  protected:
    void process(std::shared_ptr<FrameContext> ctx) override {}

    virtual void run();

    void dispatch(std::shared_ptr<FrameContext> ctx);

  private:
    std::shared_ptr<Router>     router_;
    std::shared_ptr<FrameQueue> queue_;
    std::thread                 thread_;
    std::atomic<bool>           running_{false};
    int                         cpu_affinity_;

    static constexpr std::chrono::milliseconds POP_TIMEOUT{100};
};