#pragma once

#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include "../utils/picture_db.hpp"
#include <opencv2/videoio.hpp>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

class OrbStage : public ThreadedStage {
public:
    OrbStage(std::shared_ptr<Router> router, const Config& cfg,
             std::shared_ptr<std::atomic<bool>> active_mode, int cpu_affinity = -1)
        : ThreadedStage("orb", std::move(router), cfg.get<int>("pipeline.queue_size", 32), cpu_affinity)
        , n_features_(cfg.get<int>("orb.n_features", 1000))
        , min_matches_(cfg.get<int>("orb.min_matches", 10))
        , detect_every_(cfg.get<int>("orb.detect_every", 1))
        , picture_db_path_(std::filesystem::path(cfg.get<std::string>("pictures.path", "/tmp/vision")))
        , refresh_interval_(std::chrono::seconds(cfg.get<int>("pictures.refresh_interval_s", 5)))
        , active_(std::move(active_mode))
        {}

    void init() override {
        orb_ = cv::ORB::create(n_features_);
        matcher_ = cv::BFMatcher(cv::NORM_HAMMING);
        picture_db_ = std::make_shared<PictureDB>(picture_db_path_);
        // Do an initial synchronous refresh so the first frame has a populated DB
        picture_db_->refresh(orb_);

        refresh_running_ = true;
        refresh_thread_ = std::thread([this] {
            while (refresh_running_) {
                std::this_thread::sleep_for(refresh_interval_);
                if (!refresh_running_) break;

                // Build and populate a new DB off the hot path
                auto staging = std::make_shared<PictureDB>(picture_db_path_);
                staging->refresh(orb_);

                // Swap in under lock — frame processing only holds this lock briefly
                std::lock_guard<std::mutex> lock(db_mutex_);
                picture_db_ = std::move(staging);
            }
        });
    }

    void shutdown() override {
        refresh_running_ = false;
        if (refresh_thread_.joinable())
            refresh_thread_.join();
    }

    void process(std::shared_ptr<FrameContext> ctx) override {
        ctx->flags.from_input = false;

        // Passive mode: pass frame through without detection
        if (!active_->load()) {
            ctx->flags.has_keypoints = true;
            return;
        }

        // Active mode: detect keypoints and match against reference DB
        // Skip frames to reduce CPU load — only detect every N frames.
        // has_keypoints = true routes the frame to optical_flow for continued tracking.
        if (++frame_count_ % detect_every_ != 0) {
            ctx->flags.has_keypoints = true;
            ctx->orb_result->has_matches   = false;
            return;
        }

        cv::Mat gray;
        cv::cvtColor(ctx->frame, gray, cv::COLOR_BGR2GRAY);
        orb_->detectAndCompute(gray, cv::noArray(),
                               ctx->orb_result.emplace().keypoints,
                               ctx->orb_result->descriptors);
        ctx->flags.has_keypoints = true;

        std::lock_guard<std::mutex> lock(db_mutex_);
        const auto& descriptorslist = picture_db_->descriptors();

        if (descriptorslist.empty() || ctx->orb_result->descriptors.empty()) {
            // No DB or no descriptors — proceed without matches
            ctx->orb_result->has_matches = false;
            return;
        }

        // Run BF matching against each reference picture in the DB
        for (size_t i = 0; i < descriptorslist.size(); ++i) {
            if (descriptorslist[i].empty()) continue;

            std::vector<std::vector<cv::DMatch>> knn_matches;
            matcher_.knnMatch(ctx->orb_result->descriptors, descriptorslist[i], knn_matches, 2);

            // Lowe's ratio test
            std::vector<cv::DMatch> good_matches;
            //std::vector<cv::DMatch> raw_matches;
            for (auto& m : knn_matches) {
                //if (!m.empty()) raw_matches.push_back(m[0]);
                if (m.size() == 2 && m[0].distance < 0.75f * m[1].distance)
                    good_matches.push_back(m[0]);
            }

            if (static_cast<int>(good_matches.size()) >= min_matches_) {
                ctx->matching_result.emplace();
                ctx->matching_result->matches     = std::move(good_matches);
                //ctx->matching_result->raw_matches = std::move(raw_matches);
                ctx->orb_result->object_keypoints = picture_db_->keypoints()[i];
                ctx->orb_result->object_size      = picture_db_->sizes()[i];
                ctx->orb_result->has_matches = true;
                std::cout << "Match found";
                return;
            }
        }

        // No reference picture matched well enough
        ctx->orb_result->has_matches = false;
    }


private:
    cv::Ptr<cv::ORB> orb_;
    cv::BFMatcher matcher_;
    int n_features_;
    int min_matches_;
    int detect_every_;
    int frame_count_ = -1;
    std::shared_ptr<PictureDB> picture_db_;
    std::filesystem::path picture_db_path_;
    std::chrono::seconds refresh_interval_;
    std::shared_ptr<std::atomic<bool>> active_;

    std::mutex db_mutex_;
    std::thread refresh_thread_;
    std::atomic<bool> refresh_running_{false};
};
