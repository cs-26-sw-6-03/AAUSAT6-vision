#pragma once

#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"

#include <chrono>
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <string>

class CaptureStage : public ThreadedStage {
  public:
    CaptureStage(std::shared_ptr<Router> router, const Config &cfg, int cpu_affinity = -1)
        : ThreadedStage("capture", std::move(router), 1, cpu_affinity) // queue_size=1, unused by capture as it just get whatever frame is currently being captured
          ,
          source_(cfg.require<std::string>("input.source")),
          loop_(cfg.get<bool>("input.loop", false)) {}

  protected:
    void init() override {}
    void shutdown() override {}

    //source mode, overrides run
    void process(std::shared_ptr<FrameContext>) override {}

    void run() override {
        uint64_t frame_id = 0;

        while (is_running()) {
            cv::VideoCapture cap = open_source();

            while (is_running()) {
                auto capture_start = std::chrono::steady_clock::now();

                auto ctx       = std::make_shared<FrameContext>();
                ctx->source_id = source_;
                ctx->timestamp = capture_start;

                if (!cap.read(ctx->frame) || ctx->frame.empty()) {
                    if (loop_)
                        break; // reopen source
                    return;    // source exhausted - exit naturally
                }

                auto &timing         = ctx->telemetry.per_stage["capture"];
                timing.process_start = capture_start;
                timing.process_end   = std::chrono::steady_clock::now();

                ctx->frame_id         = frame_id++;
                ctx->flags.from_input = true;

                router()->dispatch(std::move(ctx));
            }

            if (!loop_)
                break;
        }
    }

  private:
    bool is_gstreamer_pipeline(const std::string &s) const {
        return s.find('!') != std::string::npos;
    }

    cv::VideoCapture open_source() const {
        cv::VideoCapture cap;
        if (is_gstreamer_pipeline(source_)) {
            cap.open(source_, cv::CAP_GSTREAMER);
        } else {
            cap.open(source_);
        }
        if (!cap.isOpened()) {
            throw std::runtime_error("CaptureStage: failed to open source: " + source_);
        }
        return cap;
    }

    std::string source_;
    bool        loop_;
};