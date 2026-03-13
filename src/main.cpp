#include <iostream>
#include <thread>

#include "utils/config.hpp"
#include "pipeline/router.hpp"
#include "pipeline/stages/capture_stage.hpp"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: vision <config.yaml> [experiment_overlay.yaml]\n";
        return 1;
    }

    Config cfg(argv[1]);
    if (argc >= 3) {
        cfg.merge(argv[2]);
    }

    std::cout << "aausat6-vision starting\n";
    std::cout << "Config: " << cfg.path() << "\n";

    Router router;

    // --- Register stage queues ---
    int queue_size = cfg.get<int>("pipeline.queue_size", 32);

    auto optical_flow_queue = std::make_shared<FrameQueue>(queue_size);
    router.register_stage("optical_flow", optical_flow_queue);

    // --- Stub consumer: prints each received frame so we can verify capture works ---
    std::thread optical_flow_thread([&]() {
        while (true) {
            auto ctx = optical_flow_queue->pop();
            if (!ctx) break;    // queue stopped

            std::cout << "frame " << (*ctx)->frame_id
                      << "  size=" << (*ctx)->frame.cols << "x" << (*ctx)->frame.rows
                      << "  prev=" << ((*ctx)->frame_prev.empty() ? "none" : "ok")
                      << "\n";
        }
    });

    // --- Start capture ---
    CaptureStage capture(router, cfg);
    capture.start();
    capture.wait();     // blocks until source ends (or stop() is called)

    optical_flow_queue->stop();
    optical_flow_thread.join();

    std::cout << "Done.\n";
    return 0;
}
