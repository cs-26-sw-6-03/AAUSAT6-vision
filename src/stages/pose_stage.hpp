#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include <vector>
#include <iostream>

class PoseStage : public ThreadedStage
{
public:
    PoseStage(std::shared_ptr<Router> router, const Config &cfg)
        : ThreadedStage("capture", std::move(router), 1)
        , MIN_GOOD_MATCHES_(cfg.get<int>("MIN_GOOD_MATCHES", 8))
    {
    }

    void init() override
    {
    }

    void process(std::shared_ptr<FrameContext> ctx) override
    {
        if(!ctx->flags.has_keypoints){
            return;
        }

        if ((int)ctx->matching_result->matches.size() < MIN_GOOD_MATCHES_){
            return;
        }

        std::vector<cv::Point2f> ptsFrame, ptsObject;
        for(const auto& m : ctx->matching_result->matches){
            ptsFrame.push_back(ctx->[m.queryIdx].pt);
            ptsObject.push_back(ctx->orb_result->keypoints[m.trainIdx].pt);
        }
    }

private:
    int MIN_GOOD_MATCHES_

};