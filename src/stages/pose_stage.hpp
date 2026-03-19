#include "../pipeline/threadedstage.hpp"
#include "../utils/config.hpp"

class PoseStage : public ThreadedStage
{
public:
    PoseStage(std::shared_ptr<Router> router, const Config &cfg)
        : ThreadedStage("capture", std::move(router), 1)
    {
    }

    void init() override
    {
    }

    void process(std::shared_ptr<FrameContext> ctx) override
    {
        
    }

private:

};