#pragma once

/*
 * capture_stage.hpp
 *
 * Reads frames from a source and dispatches them into the pipeline via the Router.
 * Sets from_input = true on each frame so the router knows where to send it.
 *
 * input.source can be:
 *   - A file path          e.g. "data/test.mp4"       → opened with default backend
 *   - A GStreamer pipeline e.g. "v4l2src ! ..."        → opened with CAP_GSTREAMER
 *
 * GStreamer pipelines are detected by the presence of '!'.
 */

#include "../framecontext.hpp"
#include "../router.hpp"
#include "../../utils/config.hpp"

#include <opencv2/videoio.hpp>

#include <atomic>
#include <stdexcept>
#include <string>
#include <thread>

class CaptureStage {
public:
    CaptureStage(Router& router, const Config& cfg)
        : router_(router)
        , source_(cfg.require<std::string>("input.source"))
        , loop_(cfg.get<bool>("input.loop", false))
    {}

    void start() {
        running_ = true;
        thread_  = std::thread(&CaptureStage::run, this);
    }

    // Block until the source ends naturally (e.g. end of file).
    void wait() {
        if (thread_.joinable()) thread_.join();
    }

    // Signal the capture thread to stop and wait for it.
    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

private:
    bool is_gstreamer_pipeline(const std::string& s) const {
        return s.find('!') != std::string::npos;
    }

    cv::VideoCapture open_source() const {
        if (is_gstreamer_pipeline(source_)) {
            cv::VideoCapture cap(source_, cv::CAP_GSTREAMER);
            if (!cap.isOpened())
                throw std::runtime_error("CaptureStage: failed to open GStreamer pipeline: " + source_);
            return cap;
        } else {
            cv::VideoCapture cap(source_);
            if (!cap.isOpened())
                throw std::runtime_error("CaptureStage: failed to open source: " + source_);
            return cap;
        }
    }

    void run() {
        uint64_t frame_id = 0;
        cv::Mat  prev_frame;

        while (running_) {
            cv::VideoCapture cap = open_source();

            while (running_) {
                auto ctx              = std::make_shared<FrameContext>();
                ctx->source_id        = source_;
                ctx->timestamp        = std::chrono::steady_clock::now();
                ctx->flags.from_input = true;

                if (!cap.read(ctx->frame) || ctx->frame.empty()) {
                    ctx->flags.from_input = false;
                    ctx->flags.drop_frame = true;
                    router_.dispatch(std::move(ctx));
                    if (loop_) break;   // reopen source
                    running_ = false;
                    break;
                }

                ctx->frame_id   = frame_id++;
                ctx->frame_prev = prev_frame;
                prev_frame      = ctx->frame.clone();

                router_.dispatch(std::move(ctx));
            }

            if (!loop_) break;
        }
    }

    Router&           router_;
    std::string       source_;
    bool              loop_;
    std::atomic<bool> running_{false};
    std::thread       thread_;
};
