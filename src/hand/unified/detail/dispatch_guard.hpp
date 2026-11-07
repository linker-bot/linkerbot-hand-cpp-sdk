#pragma once
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <linkerbot/hand/runtime/device_runtime.hpp>
#include <mutex>
#include <optional>

namespace linkerbot::hand::unified_detail {
    // Endpoint-local timing only. Waiting never owns the Session transmitter;
    // successful sends reserve the endpoint even if their ACK later fails.
    class DispatchGuard {
    public:
        void begin_send() {
            const std::lock_guard<std::mutex> lock(mutex_);
            ++pending_sends_;
        }
        void finish_send(std::optional<runtime::Deadline> until = {}) {
            const std::lock_guard<std::mutex> lock(mutex_);
            if (pending_sends_ > 0) {
                --pending_sends_;
            }
            if (until) {
                not_before_ = std::max(not_before_, *until);
            }
            changed_.notify_all();
        }
        void reserve(runtime::Deadline until) {
            const std::lock_guard<std::mutex> lock(mutex_);
            not_before_ = std::max(not_before_, until);
            changed_.notify_all();
        }
        void wait(runtime::Deadline deadline, runtime::CancellationToken token,
                  const std::function<bool()> &valid = {}) {
            std::unique_lock<std::mutex> lock(mutex_);
            for (;;) {
                token.throw_if_cancelled();
                if (valid && !valid()) {
                    throw ClosedError("dispatch guard generation ended");
                }
                const auto now = runtime::Clock::now();
                if (now >= deadline) {
                    throw TimeoutError("dispatch guard deadline expired");
                }
                if (pending_sends_ == 0 && now >= not_before_) {
                    return;
                }
                const auto wake = pending_sends_ > 0 ? now + std::chrono::milliseconds{1} : not_before_;
                changed_.wait_until(lock, std::min({deadline, wake, now + std::chrono::milliseconds{1}}));
            }
        }

    private:
        std::mutex mutex_;
        std::condition_variable changed_;
        runtime::Deadline not_before_ = runtime::Deadline::min();
        size_t pending_sends_ = 0;
    };
}
