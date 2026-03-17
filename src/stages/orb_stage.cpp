#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"
#include "../utils/picture_db.hpp"
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <string>

class OrbStage : public ThreadedStage {
public:
    OrbStage(std::shared_ptr<Router> router, const Config& cfg)
        : ThreadedStage("orb", router, cfg.get<int>("pipeline.queue_size", 32))
        , n_features_(cfg.get<int>("orb.n_features", 1000))
        , picture_db_path_(cfg.get<std::string>("pictures.path", "/tmp/vision"))
        {}

    void init() override {
        orb_ = cv::ORB::create(n_features_);
        picture_db_ = std::make_shared<PictureDB>(picture_db_path_);
    }
    
    void process(std::shared_ptr<FrameContext> ctx) override {
        orb_->detectAndCompute(ctx->frame, cv::noArray(),
                               ctx->orb_result.emplace().keypoints,
                               ctx->orb_result->descriptors);
        ctx->flags.has_keypoints = true;
    }

    // Retrieve current sets of keypoints and descruotirs

    //Run through each pair through detection
        //BF matching
        
        //If detection is valid, break current loop and pass the ctx to the router
        

private:
    cv::Ptr<cv::ORB> orb_;
    int n_features_;
    std::shared_ptr<PictureDB> picture_db_;
    std::filesystem::path picture_db_path_;
}; 