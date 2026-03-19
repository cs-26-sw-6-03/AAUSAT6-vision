#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include <opencv2/opencv.hpp>

class OpticalFlowStage : public ThreadedStage
{
public:
    OpticalFlowStage(std::shared_ptr<Router> router, const Config &cfg)
        : ThreadedStage("OpticalFlow", router, cfg.get<int>("pipeline.queue_size", 32))
    {
    }

    void init() override
    {
    prev_detection_center = cv::Point2f(-1, -1);

    frame_idx_ = 0;
    prev_gray_.release();
    prev_kps_.clear();
    prev_desc_.release();

    }

    void process(std::shared_ptr<FrameContext> ctx) override
    {
        cv::Mat FrameMat = ctx->frame;
        cv::Mat gray;

        cv::cvtColor(FrameMat, gray, cv::COLOR_BGR2GRAY);

        if (prevGray.empty() || prev_pts_.empty())
        {
            std::vector<cv::KeyPoint> curr_kps;
            cv::Mat curr_desc;

            ctx->flags.needs_redetect = true;
            prev_pts_.clear();
            prev_pts_.reserve(curr_pts.size());
            for (const auto &kp : curr_kps)
                prev_pts_.push_back(kp.pt);

            prev_gray_ = gray.clone();
            prev_kps_ = curr_kps;
            prev_desc_ = curr_desc;
            prevGray = gray.clone();

            ctx->optical_flow_result->points_curr = curr_pts;
            ctx->optical_flow_result->points_prev = prev_pts_;
            ctx->frame = FrameMat;
            ++frame_idx_;
        }

        int det_idx = -1;
        if (ctx->pose_result->valid && prev_detection_center.x >= 0)
        {
            det_idx = prev_pts_.size();
            prev_pts_.push_back(prev_detection_center);
        }

        cv::calcOpticalFlowPyrLK(prev_gray_, gray, prev_pts_, curr_pts, status, err);

        if (det_idx >= 0 && det_idx < (int)status.size() && status[det_idx])
        {
            ctx->optical_flow_result->suggested_center = curr_pts[det_idx];
            prev_detection_center = curr_pts[det_idx];
        }
        else if (!curr_pts.empty())
        {
            cv::Point2f center(0, 0);
            for (const auto& pt : curr_pts)
                center += pt;
            center *= (1.0f / curr_pts.size());
            ctx->optical_flow_result->suggested_center = center;
            prev_detection_center = center;
        }
        std::vector<cv::Point2f> prevFiltered;
        std::vector<cv::Point2f> currFiltered;

        for (size_t i = 0; i < status.size(); i++)
        {
            if ((int)i == det_idx)
                continue;
            if (status[i])
            {
                prevFiltered.push_back(pts_to_track[i]);
                currFiltered.push_back(curr_pts[i]);
            }

            if (prevFiltered.size() < 200)
            {

                std::vector<cv::KeyPoint> kps;
                cv::Mat desc;

                ctx->flags.needs_redetect = true;

                prev_pts_.clear();
                for (const auto &kp : kps)
                    prev_pts_.push_back(kp.pt);

                prev_gray_ = gray.clone();

                ctx->optical_flow_result->points_curr = curr_pts;
                ctx->optical_flow_result->points_prev = prev_pts_;
                ctx->optical_flow_result->status = status;
                ctx->frame = FrameMat;
                ++frame_idx_;
            }
        }

        ctx->flags.needs_redetect = false;
        ctx->frame_prev = FrameMat;
        ctx->optical_flow_result->points_prev = prev_pts_;
        ctx->optical_flow_result->points_curr = curr_pts;
        ctx->optical_flow_result->status = status;

    }

private:
    std::vector<cv::Point2f> curr_pts;
    std::vector<float> err;
    std::vector<uchar> status;
    std::vector<cv::Point2f> pts_to_track;
    cv::Mat prevGray;
    std::vector<cv::Point2f> prev_pts_;
    std::vector<cv::KeyPoint> prev_kps_;
    size_t frame_idx_ = 0;
    cv::Mat prev_gray_;
    cv::Mat prev_desc_;
    cv::Point2f prev_detection_center = cv::Point2f(-1, -1);

    std::string source_;
    bool        loop_;
};