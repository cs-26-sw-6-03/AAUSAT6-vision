#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"

class PoseStage : public ThreadedStage
{
public:
    PoseStage(std::shared_ptr<Router> router, const Config &cfg)
        : ThreadedStage("capture", std::move(router), 1)  // queue_size=1, unused by capture
        , source_(cfg.require<std::string>("input.source"))
        , loop_(cfg.get<bool>("input.loop", false))
    {
    }

    void init() override
    {
    }

    void process(std::shared_ptr<FrameContext> ctx) override
    {
        
    }

private:

    std::string source_;
    bool        loop_;
};