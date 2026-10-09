#include "include/trl1_log.hpp"

namespace gtr1 {
namespace core {

void TRL1Log::record_trajectory(const MP13Blom& blom, const std::string& goal_class, double timestamp) {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    std::string id = blom.uuid();
    float marker = blom.get_outcome_marker();
    storage_[id] = Entry{blom, goal_class, timestamp, marker};

    if (!goal_class.empty()) {
        auto& list = goal_index_[goal_class];
        if (std::find(list.begin(), list.end(), id) == list.end()) {
            list.push_back(id);
        }
    }
}

void TRL1Log::record_trajectory(const gtr1::models::TRL1Entry& entry) {
    MP13Blom blom = MP13Blom::from_proto(entry.blom());
    record_trajectory(blom, entry.goal_class(), entry.timestamp());
}

std::optional<MP13Blom> TRL1Log::find_best_trajectory(float min_outcome) const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    const Entry* best_entry = nullptr;
    float best_score = min_outcome;

    for (const auto& pair : storage_) {
        const auto& entry = pair.second;
        if (entry.blom.get_outcome_marker() >= best_score) {
            best_score = entry.blom.get_outcome_marker();
            best_entry = &entry;
        }
    }

    if (best_entry) {
        return best_entry->blom;
    }
    return std::nullopt;
}

std::vector<MP13Blom> TRL1Log::find_trajectories_by_goal(const std::string& goal_class, float min_outcome) const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    std::vector<MP13Blom> result;

    auto it = goal_index_.find(goal_class);
    if (it != goal_index_.end()) {
        for (const auto& id : it->second) {
            auto sit = storage_.find(id);
            if (sit != storage_.end() && sit->second.blom.get_outcome_marker() >= min_outcome) {
                result.push_back(sit->second.blom);
            }
        }
    }
    return result;
}

std::optional<MP13Blom> TRL1Log::get_trajectory(const std::string& uuid) const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    auto it = storage_.find(uuid);
    if (it != storage_.end()) {
        return it->second.blom;
    }
    return std::nullopt;
}

bool TRL1Log::update_outcome(const std::string& uuid, float outcome_marker) {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    auto it = storage_.find(uuid);
    if (it != storage_.end()) {
        it->second.blom.set_outcome_marker(outcome_marker);
        it->second.outcome_marker = outcome_marker;
        return true;
    }
    return false;
}

bool TRL1Log::to_proto(const std::string& uuid, gtr1::models::TRL1Entry& proto) const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    auto it = storage_.find(uuid);
    if (it == storage_.end()) return false;

    it->second.blom.to_proto(*proto.mutable_blom());
    proto.set_goal_class(it->second.goal_class);
    proto.set_timestamp(it->second.timestamp);
    proto.set_outcome_marker(it->second.blom.get_outcome_marker());
    return true;
}

size_t TRL1Log::size() const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return storage_.size();
}

void TRL1Log::clear() {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    storage_.clear();
    goal_index_.clear();
}

std::vector<MP13Blom> TRL1Log::get_all_trajectories() const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    std::vector<MP13Blom> res;
    res.reserve(storage_.size());
    for (const auto& pair : storage_) {
        res.push_back(pair.second.blom);
    }
    return res;
}

} // namespace core
} // namespace gtr1