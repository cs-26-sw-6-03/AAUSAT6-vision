#pragma once
/*
 * capture_stage.cpp
 *
 * Produces frames from a source and dispatches them into the pipeline via the Router.
 * Inherits ThreadedStage for consistent lifecycle management, but overrides run()
 * since capture produces frames rather than consuming them from a queue.
 *
 * input.source can be:
 *   - A file path           e.g. "data/test.mp4"
 *   - A GStreamer pipeline  e.g. "v4l2src ! videoconvert ! appsink"  (detected by '!')
 */

 /* 
  Mental model for dropping frames in capture stage:
  Producer speed: the source fps, should be the smallest of the target or the source fps: min(source FPS, target FPS)
  Consumer speed: unlimited, the pipeline stages should not have to htink about it
  output fps: right now is just metadata, should match real ourput data rate

  Thus the capture stage should rate-limit after capturing frames.
  instead of using frame_count, it should use the wall-clock time, and drop deterministically
  ONly forwarding frames when it is time to do so, and correct for drift
 */

#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"

#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <string>

class CaptureStage : public ThreadedStage {
public:
    CaptureStage(std::shared_ptr<Router> router, const Config& cfg)
        : ThreadedStage("capture", std::move(router), 1)  // queue_size=1, unused by capture. initializer list. consumed
        , source_(cfg.require<std::string>("input.source"))
        , loop_(cfg.get<bool>("input.loop", false))
        , target_fps_(cfg.get<int>("output.fps", 30))
        , frame_period_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / target_fps_)))
        , clock_initialized_(false)
    {}

protected:
    // Override init/shutdown for resource management
    void init() override {}
    void shutdown() override {}

    // process() is never called on CaptureStage — run() is overridden directly
    void process(std::shared_ptr<FrameContext>) override {}

    // Override the worker loop entirely: produce frames instead of consuming queue
    void run() override {
        uint64_t frame_id = 0;
        cv::Mat  prev_frame;

        while (is_running()) {
            cv::VideoCapture cap = open_source();

            while (is_running()) {
                auto ctx       = std::make_shared<FrameContext>();
                ctx->source_id = source_;
                ctx->timestamp = std::chrono::steady_clock::now();

                if (!cap.read(ctx->frame) || ctx->frame.empty()) {
                    if (loop_) break;   // reopen source
                    return;             // source exhausted — exit naturally
                }
                
                // I want to dispatch/forward/emit frames when the clock* says to do so.
                auto now = std::chrono::steady_clock::now();
                
                // Indicate the clock is init
                if (!clock_initialized_) {
                    next_emit_ = now;
                    clock_initialized_ = true;
                }
                
                // Throw away frame, it is not time yet for a frame
                if (now < next_emit_) {
                    std::this_thread::sleep_until(next_emit_); // sleep on it, such that cpu does not actually read the frame just ot throw it away
                    continue;
                }

                // Advance emit clock, correcting for that drift, you know
                while (next_emit_ <= now)
                    next_emit_ += frame_period_;
                                
                
                // Emit frame
                ctx->frame_id        = frame_id++;
                ctx->frame_prev      = prev_frame;
                ctx->flags.from_input = true;
                prev_frame           = ctx->frame.clone();

                // Route the frame into the pipeline
                router()->dispatch(std::move(ctx));
            }

            if (!loop_) break;
        }
    }

private:
    bool is_gstreamer_pipeline(const std::string& s) const {
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
    int         target_fps_;
    std::chrono::steady_clock::duration   frame_period_;
    std::chrono::steady_clock::time_point next_emit_;
    bool clock_initialized_;
};