/* class OrbStage : public ThreadedStage {
public:
    OrbStage(std::shared_ptr<Router> router, const Config& cfg)
        : ThreadedStage("orb", router, cfg.get<int>("pipeline.queue_size", 32))
        , n_features_(cfg.get<int>("orb.n_features", 1000))
    {}

    void init() override {
        orb_ = cv::ORB::create(n_features_);
    }

    void process(std::shared_ptr<FrameContext> ctx) override {
        orb_->detectAndCompute(ctx->frame, cv::noArray(),
                               ctx->orb.emplace().keypoints,
                               ctx->orb->descriptors);
        ctx->flags.has_keypoints = true;
    }

private:
    cv::Ptr<cv::ORB> orb_;
    int n_features_;
}; */