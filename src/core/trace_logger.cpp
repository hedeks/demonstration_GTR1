#include <string>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <mutex>

namespace gtr1 {
namespace core {

/**
 * @brief Механизм сквозного логирования (Trace ID).
 * Позволяет проследить жизненный цикл кадра от сенсора (SLGW) 
 * до формирования сплайса (SPL1) и команды моторам (OP13).
 */
class TraceLogger {
private:
    std::mutex mtx_;

public:
    static TraceLogger& get_instance() {
        static TraceLogger instance;
        return instance;
    }

    /**
     * @brief Записывает событие с привязкой к конкретному кадру (Trace ID)
     */
    void log_event(const std::string& trace_id, const std::string& component, const std::string& message) {
        std::lock_guard<std::mutex> lock(mtx_);
        
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        
        // В реальном MVP этот лог будет публиковаться в ZeroMQ PUB сокет для Dashboard UI
        std::cout << "[" << std::put_time(std::localtime(&now_time), "%H:%M:%S") << "] "
                  << "[TraceID: " << trace_id << "] "
                  << "[" << component << "] " 
                  << message << "\n";
    }

    /**
     * @brief Генерирует уникальный Trace ID для нового кадра (внутри SLGW)
     */
    std::string generate_trace_id(uint64_t frame_number) {
        return "FRM-" + std::to_string(frame_number);
    }
};

// Макрос для удобного использования в коде
#define GTR_TRACE(trace_id, component, msg) \
    gtr1::core::TraceLogger::get_instance().log_event(trace_id, component, msg)

} // namespace core
} // namespace gtr1
