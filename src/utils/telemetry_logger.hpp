#pragma once

#include <atomic>
#include <chrono>
#include <fstream>
#include <mutex>
#include <string>

class FrameContext;

class TelemetryLogger {
  public:
    static TelemetryLogger &instance();
    void                    init(bool enabled, const std::string &path);
    void                    log_frame(const FrameContext &ctx);

  private:
    TelemetryLogger()                                   = default;
    TelemetryLogger(const TelemetryLogger &)            = delete;
    TelemetryLogger &operator=(const TelemetryLogger &) = delete;

    static long long   to_ns(const std::chrono::steady_clock::time_point &tp);
    static std::string escape_json(const std::string &s);

    std::mutex    mutex_;
    std::ofstream out_;
    bool          enabled_ = false;
    std::string   path_;
};
