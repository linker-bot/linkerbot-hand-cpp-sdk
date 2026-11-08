#include "../src/hand/unified/detail/python_health.hpp"
#include <iostream>

int main() {
    using namespace linkerbot::hand;
    unified_detail::PythonHealthMetrics metrics;
    const auto start = HealthTimePoint{};
    std::cout << '[';
    for (int i = 0; i < 6144; ++i) {
        const auto value = i < 2048 ? (i * 37) % 211 : i < 4096 ? 0 : 1000;
        metrics.completed(start, start + std::chrono::nanoseconds{value});
        HandHealth health;
        metrics.project(health);
        const auto &m = health.transaction_latency;
        if (i) {
            std::cout << ',';
        }
        std::cout << '[' << m.count << ','
                  << std::chrono::duration_cast<std::chrono::nanoseconds>(m.p50.value_or(HealthDuration{})).count()
                  << ','
                  << std::chrono::duration_cast<std::chrono::nanoseconds>(m.p95.value_or(HealthDuration{})).count()
                  << ','
                  << std::chrono::duration_cast<std::chrono::nanoseconds>(m.p99.value_or(HealthDuration{})).count()
                  << ']';
    }
    std::cout << "]\n";
}
