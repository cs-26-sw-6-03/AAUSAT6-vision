#include "telemetry_logger.hpp"

#include "../pipeline/framecontext.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

TelemetryLogger &TelemetryLogger::instance() {
	static TelemetryLogger logger;
	return logger;
}

void TelemetryLogger::init(bool enabled, const std::string &path) {
	std::lock_guard<std::mutex> lock(mutex_);

	if (out_.is_open()) {
		out_.close();
	}

	enabled_ = false;
	path_    = path;

	if (!enabled || path.empty()) {
		return;
	}

	out_.open(path, std::ios::out | std::ios::app);
	if (!out_) {
		std::cerr << "[TelemetryLogger] Failed to open telemetry file '" << path << "' — disabling telemetry" << std::endl;
		return;
	}

	enabled_ = true;
}

long long TelemetryLogger::to_ns(const std::chrono::steady_clock::time_point &tp) {
	if (tp.time_since_epoch().count() == 0) {
		return 0;
	}
	return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
}

std::string TelemetryLogger::escape_json(const std::string &s) {
	std::ostringstream oss;
	for (char c : s) {
		switch (c) {
		case '\\': oss << "\\\\"; break;
		case '"':  oss << "\\\""; break;
		case '\n': oss << "\\n"; break;
		case '\r': oss << "\\r"; break;
		case '\t': oss << "\\t"; break;
		default:
			oss << c;
			break;
		}
	}
	return oss.str();
}

void TelemetryLogger::log_frame(const FrameContext &ctx) {
	std::lock_guard<std::mutex> lock(mutex_);
	if (!enabled_ || !out_.is_open()) {
		return;
	}

	out_ << '{';
	out_ << "\"frame_id\":" << ctx.frame_id << ',';
	out_ << "\"source_id\":\"" << escape_json(ctx.source_id) << "\",";
	out_ << "\"capture_time_ns\":" << to_ns(ctx.timestamp) << ',';
	out_ << "\"dropped\":" << (ctx.flags.drop_frame ? "true" : "false") << ',';

	out_ << "\"stages\":{";
	bool first_stage = true;
	for (const auto &kv : ctx.telemetry.per_stage) {
		const auto &stage_name = kv.first;
		const auto &t          = kv.second;

		if (!first_stage) {
			out_ << ',';
		}
		first_stage = false;

		auto queue_enter_ns   = to_ns(t.queue_enter);
		auto queue_dequeue_ns = to_ns(t.queue_dequeue);
		auto process_start_ns = to_ns(t.process_start);
		auto process_end_ns   = to_ns(t.process_end);

		out_ << '"' << escape_json(stage_name) << "\":{";
		out_ << "\"queue_enter_ns\":" << queue_enter_ns << ',';
		out_ << "\"queue_dequeue_ns\":" << queue_dequeue_ns << ',';
		out_ << "\"process_start_ns\":" << process_start_ns << ',';
		out_ << "\"process_end_ns\":" << process_end_ns;

		if (queue_enter_ns > 0 && queue_dequeue_ns >= queue_enter_ns) {
			double q_ms = static_cast<double>(queue_dequeue_ns - queue_enter_ns) / 1e6;
			out_ << ",\"queue_ms\":" << q_ms;
		}
		if (process_start_ns > 0 && process_end_ns >= process_start_ns) {
			double p_ms = static_cast<double>(process_end_ns - process_start_ns) / 1e6;
			out_ << ",\"process_ms\":" << p_ms;
		}

		out_ << '}';
	}

	out_ << "}}\n";
	out_.flush();
}


