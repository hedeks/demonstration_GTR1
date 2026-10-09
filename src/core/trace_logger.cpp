#include "include/trace_logger.hpp"

namespace gtr1 {
namespace core {

void TraceLogger::log_event(const std::string& trace_id, const std::string& component, const std::string& message) {
    std::function<void(const std::string&, const std::string&, const std::string&)> sink_copy = nullptr;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);

        std::cout << "[" << std::put_time(std::localtime(&now_time), "%H:%M:%S") << "] "
                  << "[TraceID: " << trace_id << "] "
                  << "[" << component << "] "
                  << message << "\n";

        sink_copy = telemetry_sink_;
    }

    if (sink_copy) {
        sink_copy(trace_id, component, message);
    }
}

} // namespace core
} // namespace gtr1