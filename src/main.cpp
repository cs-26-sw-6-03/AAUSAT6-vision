#include <iostream>
#include <csignal>
#include <atomic>

#include <opencv2/core.hpp>
#include "pipeline/pipeline.hpp"

// --- Stages ---
#include "stages/capture_stage.hpp"
#include "stages/orb_stage.hpp"
#include "stages/optical_flow_stage.hpp"
#include "stages/pose_stage.hpp"
#include "stages/ed_ransac_stage.hpp"
#include "stages/output_stage.hpp"

static std::atomic<bool> g_shutdown{false};

static void signal_handler(int) {
    g_shutdown.store(true);
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: vision <config.yaml> [experiment_overlay.yaml]\n";
        return 1;
    }

    // --- Config ---
    Config cfg(argv[1]);
    if (argc >= 3) {
        cfg.merge(argv[2]);
    }
    std::cout << "aausat6-vision starting\n";
    std::cout << "Config:  " << cfg.path() << "\n";

    // --- Signal handling ---
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Disable OpenCV's internal thread pools — the pipeline already runs one thread per stage,
    // so OpenCV workers would only cause over-subscription and cache thrashing.
    cv::setNumThreads(1);

    // --- Build pipeline ---
    Pipeline pipeline(cfg);

    // Shared flag: OrbStage starts active; OpticalFlowStage flips it to passive
    // once a valid detection is found, and back to active when tracking is lost.
    auto orb_mode = std::make_shared<std::atomic<bool>>(true);

    // Stage order: capture -> orb -> optical_flow -> pose -> ransac -> output
    pipeline.add_stage(std::make_shared<CaptureStage>     (pipeline.router(), cfg));
    pipeline.add_stage(std::make_shared<OrbStage>         (pipeline.router(), cfg, orb_mode));
    pipeline.add_stage(std::make_shared<OpticalFlowStage> (pipeline.router(), cfg, orb_mode));
    pipeline.add_stage(std::make_shared<PoseStage>        (pipeline.router(), cfg));
    pipeline.add_stage(std::make_shared<EdRansacStage>    (pipeline.router(), cfg));
    pipeline.add_stage(std::make_shared<OutputStage>      (pipeline.router(), cfg));

    pipeline.start();

    // --- Run until SIGINT / SIGTERM ---
    std::cout << "Running. Press Ctrl+C to stop.\n";
    while (!g_shutdown.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "\nShutting down...\n";
    pipeline.stop();

    return 0;
}
