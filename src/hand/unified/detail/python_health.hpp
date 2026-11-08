#pragma once

#include <deque>
#include <linkerbot/hand/health.hpp>
#include <mutex>

namespace linkerbot::hand::unified_detail {
    // Device-lifetime logical completions. No transport ownership or scheduling.
    class PythonHealthMetrics {
    public:
        PythonHealthMetrics();
        void completed(HealthTimePoint started, HealthTimePoint finished);
        void project(HandHealth &health) const;
        void clear();

    private:
        mutable std::mutex mutex_;
        std::deque<HealthDuration> latencies_;
        std::vector<HealthDuration> ordered_;
        mutable std::optional<LatencyMetrics> cached_;
        std::optional<HealthTimePoint> last_transaction_;
    };
}
