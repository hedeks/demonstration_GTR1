#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <shared_mutex>
#include <mutex>
#include <random>
#include <sstream>
#include <iomanip>
#include <regex>

#include "gtr1_structures.pb.h"

namespace gtr1 {
namespace core {

/**
 * @brief Базовый абстрактный класс Vali (Valence-Instance).
 * Фундамент ООП-модели архитектуры GTR1.
 * Все структуры памяти (MP11-MP13) наследуют ValiBase.
 * Обеспечивает строгий жизненный цикл валентностей, UUIDv4 и потокобезопасность.
 */
class ValiBase {
protected:
    std::string uuid_;
    bool is_active_{true};
    std::vector<std::string> valences_;
    mutable std::shared_mutex mtx_;

public:
    explicit ValiBase(std::string id = "", size_t num_valences = 0)
        : uuid_(id.empty() ? generate_uuid() : std::move(id)),
          is_active_(true),
          valences_(num_valences, "") {}

    virtual ~ValiBase() = default;

    // --- Копирование и перемещение с потокобезопасностью ---

    ValiBase(const ValiBase& other) {
        std::shared_lock<std::shared_mutex> lock(other.mtx_);
        uuid_ = other.uuid_;
        is_active_ = other.is_active_;
        valences_ = other.valences_;
    }

    ValiBase& operator=(const ValiBase& other) {
        if (this != &other) {
            std::scoped_lock lock(mtx_, other.mtx_);
            uuid_ = other.uuid_;
            is_active_ = other.is_active_;
            valences_ = other.valences_;
        }
        return *this;
    }

    ValiBase(ValiBase&& other) noexcept {
        std::unique_lock<std::shared_mutex> lock(other.mtx_);
        uuid_ = std::move(other.uuid_);
        is_active_ = other.is_active_;
        valences_ = std::move(other.valences_);
    }

    ValiBase& operator=(ValiBase&& other) noexcept {
        if (this != &other) {
            std::scoped_lock lock(mtx_, other.mtx_);
            uuid_ = std::move(other.uuid_);
            is_active_ = other.is_active_;
            valences_ = std::move(other.valences_);
        }
        return *this;
    }

    // --- Доступ к UUID и активности ---

    std::string uuid() const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        return uuid_;
    }

    std::string get_uuid() const {
        return uuid();
    }

    void set_uuid(const std::string& id) {
        std::unique_lock<std::shared_mutex> lock(mtx_);
        uuid_ = id;
    }

    bool is_active() const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        return is_active_;
    }

    void set_active(bool active) {
        std::unique_lock<std::shared_mutex> lock(mtx_);
        is_active_ = active;
    }

    // --- Методология валентностей ---

    /**
     * @brief Проверяет наличие открытых (незаполненных) валентностей.
     * Открытая валентность = пустая строка "".
     */
    bool has_open_valences() const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        return std::any_of(valences_.begin(), valences_.end(),
                           [](const std::string& v) { return v.empty(); });
    }

    /**
     * @brief Количество открытых валентностей.
     */
    size_t open_valences_count() const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        return std::count_if(valences_.begin(), valences_.end(),
                             [](const std::string& v) { return v.empty(); });
    }

    /**
     * @brief Общее количество слотов валентностей.
     */
    size_t total_valences_count() const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        return valences_.size();
    }

    /**
     * @brief Закрывает валентность по индексу целевым UUID.
     * @throws std::invalid_argument если target_uuid пустой.
     * @throws std::out_of_range если индекс выходит за границы.
     */
    void close_valence(size_t index, const std::string& target_uuid) {
        if (target_uuid.empty()) {
            throw std::invalid_argument("Cannot close valence with empty UUID");
        }
        std::unique_lock<std::shared_mutex> lock(mtx_);
        if (index >= valences_.size()) {
            throw std::out_of_range("Valence index out of bounds: " + std::to_string(index) +
                                    " (total: " + std::to_string(valences_.size()) + ")");
        }
        valences_[index] = target_uuid;
    }

    /**
     * @brief Открывает валентность (сбрасывает слот в пустую строку "").
     * @throws std::out_of_range если индекс выходит за границы.
     */
    void open_valence(size_t index) {
        std::unique_lock<std::shared_mutex> lock(mtx_);
        if (index >= valences_.size()) {
            throw std::out_of_range("Valence index out of bounds: " + std::to_string(index));
        }
        valences_[index] = "";
    }

    /**
     * @brief Получает UUID валентности по индексу.
     * @throws std::out_of_range если индекс выходит за границы.
     */
    std::string get_valence(size_t index) const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        if (index >= valences_.size()) {
            throw std::out_of_range("Valence index out of bounds: " + std::to_string(index));
        }
        return valences_[index];
    }

    /**
     * @brief Проверяет, открыт ли конкретный слот валентности.
     */
    bool is_valence_open(size_t index) const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        if (index >= valences_.size()) {
            throw std::out_of_range("Valence index out of bounds: " + std::to_string(index));
        }
        return valences_[index].empty();
    }

    /**
     * @brief Возвращает снимок вектора валентностей.
     */
    std::vector<std::string> get_valences() const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        return valences_;
    }

    /**
     * @brief Добавляет новый открытый слот валентности.
     * @return Индекс добавленного слота.
     */
    size_t add_valence_slot() {
        std::unique_lock<std::shared_mutex> lock(mtx_);
        valences_.push_back("");
        return valences_.size() - 1;
    }

    /**
     * @brief Изменяет размер массива валентностей.
     */
    void resize_valences(size_t count) {
        std::unique_lock<std::shared_mutex> lock(mtx_);
        valences_.resize(count, "");
    }

    // --- Сериализация в/из Protobuf DTO ---

    void to_proto(gtr1::models::ValiBase& proto) const {
        std::shared_lock<std::shared_mutex> lock(mtx_);
        proto.set_uuid(uuid_);
        proto.set_is_active(is_active_);
        proto.clear_valences();
        for (const auto& v : valences_) {
            proto.add_valences(v);
        }
    }

    void from_proto(const gtr1::models::ValiBase& proto) {
        std::unique_lock<std::shared_mutex> lock(mtx_);
        uuid_ = proto.uuid();
        is_active_ = proto.is_active();
        valences_.clear();
        for (int i = 0; i < proto.valences_size(); ++i) {
            valences_.push_back(proto.valences(i));
        }
    }

    // --- Статические утилиты: UUIDv4 ---

    /**
     * @brief Генерация канонического UUIDv4 по RFC 4122.
     */
    static std::string generate_uuid() {
        thread_local std::random_device rd;
        thread_local std::mt19937_64 gen(rd());
        thread_local std::uniform_int_distribution<uint64_t> dis;

        uint64_t ab = dis(gen);
        uint64_t cd = dis(gen);

        // Версия 4 (0100 в старших 4 битах time_hi_and_version)
        ab = (ab & 0xFFFFFFFFFFFF0FFFULL) | 0x0000000000004000ULL;
        // Вариант 1 (10 в старших 2 битах clock_seq_hi_and_reserved)
        cd = (cd & 0x3FFFFFFFFFFFFFFFULL) | 0x8000000000000000ULL;

        std::stringstream ss;
        ss << std::hex << std::setfill('0');
        ss << std::setw(8) << (ab >> 32);
        ss << "-";
        ss << std::setw(4) << ((ab >> 16) & 0xFFFF);
        ss << "-";
        ss << std::setw(4) << (ab & 0xFFFF);
        ss << "-";
        ss << std::setw(4) << (cd >> 48);
        ss << "-";
        ss << std::setw(12) << (cd & 0xFFFFFFFFFFFFULL);
        return ss.str();
    }

    /**
     * @brief Валидация формата UUID (поддерживает UUIDv4 и опциональные префиксы вида OBJ-, OPRN-, BLOM-).
     */
    static bool is_valid_uuid(const std::string& str) {
        static const std::regex uuid_pattern(
            R"(^([A-Za-z0-9_]+-)?[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-4[0-9a-fA-F]{3}-[89abAB][0-9a-fA-F]{3}-[0-9a-fA-F]{12}$)");
        return std::regex_match(str, uuid_pattern);
    }
};

} // namespace core
} // namespace gtr1