/*
 * pipeline.cpp
 *
 * Implementation of pipeline class.
 */

#include "pipeline.hpp"
#include <iostream>
#include <stdexcept>

Pipeline::Pipeline(const Config &config)
    : router_(std::make_shared<Router>()),
      config_(config) {}

void Pipeline::add_stage(std::shared_ptr<ThreadedStage> stage) {
    if (running_) {
        throw std::runtime_error("Pipeline: cannot add stages while running");
    }
    const std::string &name = stage->name();
    if (stages_.count(name)) {
        throw std::runtime_error("Pipeline: duplicate stage name '" + name + "'");
    }
    router_->register_stage(name, stage->queue());
    stages_[name] = stage;
    stage_order_.push_back(std::move(stage));
}

void Pipeline::start() {
    if (running_)
        return;
    if (stage_order_.empty()) {
        throw std::runtime_error("Pipeline: no stages registered");
    }
    for (auto &stage : stage_order_) {
        stage->start();
    }
    running_ = true;
    std::cout << "Pipeline started with " << stage_order_.size() << " stage(s)\n";
}

void Pipeline::stop() {
    if (!running_)
        return;
    // Stop in reverse order so upstream stages drain before downstream
    for (auto it = stage_order_.rbegin(); it != stage_order_.rend(); ++it) {
        (*it)->stop();
    }
    running_ = false;
    std::cout << "Pipeline stopped\n";
}

void Pipeline::push(std::shared_ptr<FrameContext> ctx,
                    const std::string            &target_stage) {
    if (!running_) {
        throw std::runtime_error("Pipeline: not running");
    }
    if (!target_stage.empty()) {
        auto it = stages_.find(target_stage);
        if (it == stages_.end()) {
            throw std::runtime_error("Pipeline: unknown stage '" + target_stage + "'");
        }
        it->second->enqueue(std::move(ctx));
    } else {
        if (stage_order_.empty())
            return;
        stage_order_.front()->enqueue(std::move(ctx));
    }
}