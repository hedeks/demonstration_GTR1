#pragma once

#include <string>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <functional>

namespace gtr1 {
namespace core {

/**
 * @brief TraceLogger: Механизм сквозного логирования (Trace ID).
 * Позволяет проследить жизненный цикл кадра от сенсора (SLGW) 
 * до формирования сплайса (SPL1), планирования (PRD1) и команды моторам (OP13).
 * Потокобезопасен (std::mutex).
 */
class TraceLogger {
private:
    std::mutex mtx_;
    std::function<void(const std::string&, const std::string&, const std::string&)> telemetry_sink_{nullptr};

    TraceLogger() = default;

public:
    static TraceLogger& get_instance() {
        static TraceLogger instance;
        return instance;
    }

    /**
     * @brief Назначает пользовательский callback для пересылки логов в ZeroMQ / WebSocket телеметрию.
     */
    void set_telemetry_sink(std::function<void(const std::string&, const std::string&, const std::string&)> sink) {
        std::lock_guard<std::mutex> lock(mtx_);
        telemetry_sink_ = std::move(sink);
    }

    /**
     * @brief Записывает событие с привязкой к конкретному кадру (Trace ID).
     */
    void log_event(const std::string& trace_id, const std::string& component, const std::string& message);

    /**
     * @brief Генерирует уникальный Trace ID для нового кадра (внутри SLGW).
     */
    static std::string generate_trace_id(uint64_t frame_number) {
        return "FRM-" + std::to_string(frame_number);
    }
};

} // namespace core
} // namespace gtr1

// Макрос для удобного сквозного логирования в компонентах
#define GTR_TRACE(trace_id, component, msg) gtr1::core::TraceLogger::get_instance().log_event(trace_id, component, msg)
