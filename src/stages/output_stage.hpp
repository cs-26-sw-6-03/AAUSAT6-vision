#pragma once

 
/*
 * output_stage.hpp
 *
 * Terminal stage — receives processed frames and pushes them out.
 * Mode is selected via config:
 *
 *   output.mode: file       → GStreamer filesink (e.g. MP4/MKV)
 *   output.mode: rtp        → RTP/UDP stream via GStreamer
 *
 * Config keys:
 *
 *   output.mode             file | rtp | file_vpu | rtp_vpu
 *   output.width            crop/resize target width  (0 = use input dimensions)
 *   output.height           crop/resize target height (0 = use input dimensions)
 *   output.fps              frames per second (default: 30)
 *   output.crop             if true, center-crop instead of resize
 *
 *   # file mode
 *   output.file.path        output file path, e.g. "out/recording.mp4"
 *   output.file.encoder     GStreamer encoder element, e.g. "x264enc"
 *   output.file.muxer       GStreamer muxer element,   e.g. "mp4mux"
 *
 *   # rtp mode
 *   output.rtp.host         destination IP
 *   output.rtp.port         destination UDP port
 *   output.rtp.encoder      GStreamer encoder element, e.g. "x264enc"
 *
 *   # vpu-accelerated modes (for Debix model B)
 *   output.file.encoder     e.g. "vpuenc_h264"
 *   output.file.muxer       e.g. "h264parse"
 *   output.rtp.encoder      e.g. "vpuenc_h264"
 */

#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include "../utils/telemetry_logger.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <opencv2/imgproc.hpp>

#include <stdexcept>
#include <string>
#include <sstream>
#include <iostream> // For printing to stdout

class OutputStage : public ThreadedStage {
public:
    OutputStage(std::shared_ptr<Router> router, const Config& cfg, int cpu_affinity = -1)
        : ThreadedStage("output", std::move(router), cfg.get<int>("pipeline.queue_size", 32), cpu_affinity)
        , mode_(cfg.require<std::string>("output.mode"))
        , fps_(cfg.get<int>("output.fps", 30))
        , bitrate_(cfg.get<int>("output.bitrate", 2000))
        , width_(cfg.get<int>("output.width", 0)) // default: use input
        , height_(cfg.get<int>("output.height", 0)) // default: use input
        , crop_(cfg.get<bool>("output.crop", false))
        {
            //std::string mode = cfg.require<std::string>("output.mode");

            // set up mode-specific configs
            if (mode_ == "file") {
                file_path_    = cfg.require<std::string>("output.file.path");
                file_encoder_ = cfg.get<std::string>("output.file.encoder", "x264enc"); // vpuenc_h264
                file_muxer_   = cfg.get<std::string>("output.file.muxer",   "mp4mux"); // tror der er en for imx8mp
            } else if (mode_ == "rtp") {
                rtp_host_     = cfg.require<std::string>("output.rtp.host");
                rtp_port_     = cfg.require<int>("output.rtp.port");
                rtp_encoder_  = cfg.get<std::string>("output.rtp.encoder", "x264enc"); //vpuenc_h264
            } else {
                throw std::runtime_error("OutputStage: unknown mode '" + mode_ + "'. Expected 'file' or 'rtp'");
            }
        }

    void init() override {
        gst_init(nullptr, nullptr); // initialize GStreamer liib
        pipeline_ = build_pipeline(); // build pipeline from config
        appsrc_   = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "src")); // get appsrc element for pushing frames
        if (!appsrc_) {
            throw std::runtime_error("OutputStage: could not find appsrc element 'src'");
        }
        gst_element_set_state(pipeline_, GST_STATE_PLAYING); // set pipeline to playing so it's ready to receive frames

        std::cout << "[OutputStage] mode="     << mode_
                  << " width="    << width_
                  << " height="   << height_
                  << " fps="      << fps_
                  << " encoder="  << file_encoder_
                  << " muxer="    << file_muxer_
                  << " path="     << file_path_
                  << " rtp_host=" << rtp_host_
                  << " rtp_port=" << rtp_port_
                  << "\n";
    }

    void shutdown() override {
        // Shutdown resource managment
        if (pipeline_) {
            gst_element_send_event(pipeline_, gst_event_new_eos());
            // Wait for EOS to propagate
            GstBus* bus = gst_element_get_bus(pipeline_);
            gst_bus_timed_pop_filtered(bus, GST_SECOND * 5, static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
            gst_object_unref(bus);
            gst_element_set_state(pipeline_, GST_STATE_NULL);
            gst_object_unref(pipeline_);
            pipeline_ = nullptr;
            appsrc_   = nullptr;
        }
    }

    void process(std::shared_ptr<FrameContext> ctx) override {
        if (!ctx || ctx->frame.empty()) return; // if frame is empty, skip processing

        cv::Mat frame = ctx->frame; // make alias for frame in context (we do not like hte -> syntax)

        // IMPORTANT
        // latch the width and height on the actual frame and configure teh appsrc and caps + start the pipeline from the first frame
        // This should hopefully remove the FRAMDATA ONLY BEING ONE FREAKING PIXEL TALL issue
        if (!caps_set_) {
            actual_width_  = frame.cols;
            actual_height_ = frame.rows;
 
            std::string caps_str =
                "video/x-raw,format=BGR"
                ",width="     + std::to_string(actual_width_)  +
                ",height="    + std::to_string(actual_height_) +
                ",framerate=" + std::to_string(fps_) + "/1";
 
            GstCaps* caps = gst_caps_from_string(caps_str.c_str());
            gst_app_src_set_caps(appsrc_, caps); // Set the caps for the Gstreamer pipeline
            gst_caps_unref(caps);
 
            gst_element_set_state(pipeline_, GST_STATE_PLAYING);
 
            caps_set_ = true;
            std::cout << "[OutputStage] caps latched: "
                      << actual_width_ << "x" << actual_height_
                      << " @ " << fps_ << "fps\n";
        }

        // Drop frames that don't match the latched dimensions to avoid pushing
        // mismatched strides into GStreamer.
        if (frame.cols != actual_width_ || frame.rows != actual_height_) {
            std::cerr << "[OutputStage] frame size mismatch: got "
                      << frame.cols << "x" << frame.rows
                      << ", expected " << actual_width_ << "x" << actual_height_
                      << " — dropping\n";
            return;
        }









        /* if (frame.data.cols != width_ || frame.data.rows != height_) {
            std::cerr << "[GstreamerFileOutput] Frame size mismatch. Expected "
                    << width_ << "x" << height_ << ", got "
                    << frame.data.cols << "x" << frame.data.rows << "\n";
        } */

        // GStreamer expects BGR, which is OpenCV's default — no conversion needed
        gsize size = static_cast<gsize>(frame.total() * frame.elemSize()); // calc frame size in bytes
        GstBuffer* buf = gst_buffer_new_allocate(nullptr, size, nullptr); // alloc gst buffer for frame data

        GstMapInfo map; // map buffer memory so we can write frame data into it
        gst_buffer_map(buf, &map, GST_MAP_WRITE);
        std::memcpy(map.data, frame.data, size); // copy frame data into gst buffer
        gst_buffer_unmap(buf, &map); //unmap buffer memory, im done writing data into it

        // Timestamp
        GST_BUFFER_PTS(buf) = gst_util_uint64_scale(frame_count_, GST_SECOND, fps_); // presentation timestamp based on framecontext
        GST_BUFFER_DURATION(buf) = gst_util_uint64_scale(1, GST_SECOND, fps_); //frame duration based on fps config
        //++frame_count_; // increment frame count for next timestamp

        GstFlowReturn ret = gst_app_src_push_buffer(appsrc_, buf); // push buffer into pipeline via appsrc
        if (ret != GST_FLOW_OK) {
            throw std::runtime_error("OutputStage: gst_app_src_push_buffer failed: " + std::to_string(ret));
        }

        // Mark frame as fully handled so the router does not re-dispatch it.
        if (!ctx->telemetry.logged) {
            TelemetryLogger::instance().log_frame(*ctx);
            ctx->telemetry.logged = true;
        }
        ctx->flags.from_input = false;
        ctx->flags.drop_frame = true;

        // Progress indicator in the terminal.
        if (frame_count_ % 30 == 0) { // Print to stdout for some kind of feedback
            std::cout << "[output] pushed " << frame_count_ << " frames" << std::endl;
        }

        ++frame_count_; // increment frame count for next timestamp
    }

private:
    GstElement* build_pipeline() {
        std::string desc = build_pipeline_string(); // makes a freaking string
        std::cout << "[OutputStage] pipeline: " << desc << "\n";
        GError* err = nullptr; // see if it fails or something
        GstElement* pipe = gst_parse_launch(desc.c_str(), &err); 
        if (!pipe || err) {
            std::string msg = err ? err->message : "unknown error";
            if (err) g_error_free(err);
            throw std::runtime_error("OutputStage: failed to build GStreamer pipeline: " + msg);
        }
        return pipe;
    }

   std::string build_pipeline_string() const {
        // Caps for the appsrc — raw BGR frames from OpenCV
        //std::string caps =
        //    "video/x-raw,format=BGR"
        //    ",width=" + std::to_string(width_ > 0 ? width_ : 1920) + // default to 1080p if not specified
        //    ",height=" + std::to_string(height_ > 0 ? height_ : 1080) +
        //    ",framerate=" + std::to_string(fps_) + "/1"; // e.g. "video/x-raw,format=BGR,width=1920,height=1080,framerate=30/1"

        std::ostringstream p;
        //p << "appsrc name=src is-live=true block=true format=time caps=\"" << caps << "\"";
        p << "appsrc name=src is-live=true block=true format=time";
        p << " ! videoconvert";


        // RESIZE to scale down inside the gstreamer if target dimentions are too large
        // TODO: remove maybe?
        if (width_ > 0 && height_ > 0) {
            p << " ! videoscale"
            << " ! video/x-raw,width=" << width_ << ",height=" << height_;
        }

        if (mode_ == "file") {
            p << " ! " << file_encoder_ << " bitrate=" << bitrate_;
            p << " ! " << file_muxer_;
            p << " ! filesink location=\"" << file_path_ << "\"";
        } else {
            p << " ! " << rtp_encoder_ << " bitrate=" << bitrate_ << " tune=zerolatency";
            p << " ! rtph264pay"; // // config-interval=1 pt=96
            p << " ! udpsink host=" << rtp_host_ << " port=" << rtp_port_;
        }

        return p.str();
    }

    // Config
    std::string mode_; // file or rtp
    int         fps_;
    int         bitrate_;
    int         width_;
    int         height_;
    bool        crop_;

    // File mode
    std::string file_path_;
    std::string file_encoder_;
    std::string file_muxer_;

    // RTP mode
    std::string rtp_host_; // destination IP
    int         rtp_port_    = 0; // likely somehting like port 5000
    std::string rtp_encoder_; // vpuenc_h264 or the generic

    // GStreamer
    GstElement* pipeline_    = nullptr; // string of elements built in init()
    GstAppSrc*  appsrc_      = nullptr; // cached pointer to appsrc for pushing frames
    uint64_t    frame_count_ = 0; // for timestamping frames in output


    // Runtime-latched caps state
    bool        caps_set_      = false;
    int         actual_width_  = 0;
    int         actual_height_ = 0;
};