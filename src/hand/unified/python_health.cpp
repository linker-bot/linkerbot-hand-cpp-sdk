#include "detail/python_health.hpp"
#include "detail/backend.hpp"
#include <algorithm>
#include <linkerbot/hand/errors.hpp>

namespace linkerbot::hand::unified_detail {
    std::unique_ptr<DeviceBackend>
    make_backend_with_python_health(const HandDeviceConfig &config, const HandDevice::TransportFactory &factory,
                                    const std::shared_ptr<transport::Session> &session,
                                    const std::shared_ptr<PythonHealthMetrics> &metrics) {
        auto backend = make_backend(config, factory, session);
        backend->set_python_health(metrics);
        return backend;
    }
    PythonHealthMetrics::PythonHealthMetrics() { ordered_.reserve(kHealthLatencyWindowCapacity + 1); }
    void PythonHealthMetrics::completed(HealthTimePoint started, HealthTimePoint finished) {
        if (finished < started) {
            throw ValidationError("logical transaction duration must be non-negative");
        }
        const std::lock_guard<std::mutex> lock(mutex_);
        latencies_.push_back(finished - started);
        if (latencies_.size() > kHealthLatencyWindowCapacity) {
            ordered_.erase(std::lower_bound(ordered_.begin(), ordered_.end(), latencies_.front()));
            latencies_.pop_front();
        }
        ordered_.insert(std::upper_bound(ordered_.begin(), ordered_.end(), finished - started), finished - started);
        cached_.reset();
        last_transaction_ = finished;
    }
    void PythonHealthMetrics::project(HandHealth &health) const {
        const std::lock_guard<std::mutex> lock(mutex_);
        health.last_successful_transaction = last_transaction_;
        if (!cached_) {
            LatencyMetrics metrics;
            metrics.count = ordered_.size();
            if (!ordered_.empty()) {
                const auto percentile = [&](size_t percentage) {
                    const auto scaled = (ordered_.size() - 1) * percentage;
                    const auto lower = scaled / 100;
                    const auto remainder = scaled % 100;
                    // Python round(): nearest index, with exact halves going to even.
                    const auto index = lower + (remainder > 50 || (remainder == 50 && lower % 2 != 0));
                    return ordered_[index];
                };
                metrics.p50 = percentile(50);
                metrics.p95 = percentile(95);
                metrics.p99 = percentile(99);
            }
            cached_ = metrics;
        }
        health.transaction_latency = cached_.value_or(LatencyMetrics{});
    }
    void PythonHealthMetrics::clear() {
        const std::lock_guard<std::mutex> lock(mutex_);
        latencies_.clear();
        ordered_.clear();
        cached_.reset();
        last_transaction_.reset();
    }
}
