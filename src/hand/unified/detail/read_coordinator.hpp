#pragma once

#include <chrono>
#include <condition_variable>
#include <deque>
#include <exception>
#include <functional>
#include <linkerbot/hand/errors.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace linkerbot::hand::unified_detail {
    // Coordinates device intents, not CAN sends. The Session remains the only
    // physical sender. Each caller owns its wait budget; leaving a wait never
    // cancels a transaction shared with another caller or a poller.
    template <typename Value> class ReadCoordinator {
    public:
        using Clock = std::chrono::steady_clock;
        explicit ReadCoordinator(size_t capacity)
            : capacity_(checked_capacity(capacity)),
              closed_error_(std::make_exception_ptr(ClosedError("read generation ended"))), worker_([this] { run(); }) {
        }
        ~ReadCoordinator() {
            invalidate();
            {
                const std::lock_guard<std::mutex> lock(mutex_);
                stopping_ = true;
            }
            changed_.notify_all();
            worker_.join();
        }
        ReadCoordinator(const ReadCoordinator &) = delete;
        ReadCoordinator &operator=(const ReadCoordinator &) = delete;

        Value read(unsigned key, Clock::time_point deadline, std::function<Value()> execute, bool polling = false) {
            std::unique_lock<std::mutex> lock(mutex_);
            if (!accepting_) {
                throw ClosedError("read generation ended");
            }
            if (Clock::now() >= deadline) {
                throw TimeoutError("active read deadline expired");
            }
            if (!polling && waiters_ == capacity_) {
                throw QueueFullError("active read waiter capacity is full");
            }
            auto found = entries_.find(key);
            std::shared_ptr<Entry> entry;
            if (found == entries_.end()) {
                entry = std::make_shared<Entry>();
                entry->execute = std::move(execute);
                // Install the queue first so allocation failure cannot leave an
                // entry that no worker can ever claim.
                queue_.push_back({key, entry});
                try {
                    entries_.emplace(key, entry);
                } catch (...) {
                    queue_.pop_back();
                    throw;
                }
            } else {
                entry = found->second;
            }
            const auto generation = generation_;
            if (!polling) {
                ++waiters_;
            }
            ++entry->waiters;
            changed_.notify_all();
            bool completed = true;
            if (deadline == Clock::time_point::max()) {
                changed_.wait(lock, [&] { return entry->done; });
            } else {
                completed = changed_.wait_until(lock, deadline, [&] { return entry->done; });
            }
            if (!polling && generation == generation_) {
                --waiters_;
            }
            --entry->waiters;
            if (!entry->done && !entry->claimed && entry->waiters == 0) {
                entries_.erase(key);
                for (auto it = queue_.begin(); it != queue_.end(); ++it) {
                    if (it->second == entry) {
                        queue_.erase(it);
                        break;
                    }
                }
            }
            if (!completed) {
                throw TimeoutError("active read did not complete before its deadline");
            }
            if (entry->error) {
                rethrow_error(entry->error);
            }
            if (!entry->value) {
                throw InternalError("read completed without a value");
            }
            return *entry->value;
        }
        // Must precede runtime cancellation/draining. No late result can complete
        // a waiter from an invalidated generation.
        void invalidate() {
            const auto error = closed_error_;
            const std::lock_guard<std::mutex> lock(mutex_);
            accepting_ = false;
            ++generation_;
            waiters_ = 0;
            for (auto &[key, entry] : entries_) {
                (void)key;
                entry->error = error;
                entry->done = true;
            }
            entries_.clear();
            queue_.clear();
            changed_.notify_all();
        }
        void drain() {
            std::unique_lock<std::mutex> lock(mutex_);
            changed_.wait(lock, [&] { return !active_; });
        }
        void activate() {
            const std::lock_guard<std::mutex> lock(mutex_);
            if (active_ || !entries_.empty()) {
                throw LifecycleError("cannot activate undrained reads");
            }
            accepting_ = true;
        }

    private:
        [[noreturn]] static void rethrow_error(const std::exception_ptr &error) {
            // Public error normalization enriches context. Each reader needs
            // its own exception object, including across invalidated generations.
            try {
                std::rethrow_exception(error);
            }
#define LINKERBOT_COPY_READ_ERROR(Type)                                                                                \
    catch (const Type &failure) {                                                                                      \
        throw Type(failure);                                                                                           \
    }
            LINKERBOT_COPY_READ_ERROR(ResponseTimeoutError)
            LINKERBOT_COPY_READ_ERROR(IdentityProtocolError)
            LINKERBOT_COPY_READ_ERROR(ProtocolError)
            LINKERBOT_COPY_READ_ERROR(ClosedError)
            LINKERBOT_COPY_READ_ERROR(TimeoutError)
            LINKERBOT_COPY_READ_ERROR(TransportError)
            LINKERBOT_COPY_READ_ERROR(DeviceNotReadyError)
            LINKERBOT_COPY_READ_ERROR(IdentityMismatchError)
            LINKERBOT_COPY_READ_ERROR(UnsupportedOperationError)
            LINKERBOT_COPY_READ_ERROR(UnsupportedFirmwareError)
            LINKERBOT_COPY_READ_ERROR(ValidationError)
            LINKERBOT_COPY_READ_ERROR(QueueFullError)
            LINKERBOT_COPY_READ_ERROR(DeviceFaultError)
            LINKERBOT_COPY_READ_ERROR(InternalError)
            LINKERBOT_COPY_READ_ERROR(HandError)
#undef LINKERBOT_COPY_READ_ERROR
        }
        static size_t checked_capacity(size_t capacity) {
            if (capacity == 0) {
                throw ValidationError("read waiter capacity must be positive");
            }
            return capacity;
        }
        struct Entry {
            std::function<Value()> execute;
            std::optional<Value> value;
            std::exception_ptr error;
            bool done = false;
            bool claimed = false;
            size_t waiters = 0;
        };
        void run() {
            std::unique_lock<std::mutex> lock(mutex_);
            while (!stopping_) {
                changed_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
                if (stopping_) {
                    break;
                }
                auto [key, entry] = std::move(queue_.front());
                queue_.pop_front();
                active_ = true;
                entry->claimed = true;
                lock.unlock();
                std::optional<Value> value;
                std::exception_ptr error;
                try {
                    value.emplace(entry->execute());
                } catch (...) {
                    error = std::current_exception();
                }
                lock.lock();
                if (!entry->done) {
                    try {
                        entry->value = std::move(value);
                        entry->error = error;
                    } catch (...) {
                        entry->error = std::current_exception();
                    }
                    entry->done = true;
                }
                if (const auto it = entries_.find(key); it != entries_.end() && it->second == entry) {
                    entries_.erase(it);
                }
                active_ = false;
                changed_.notify_all();
            }
        }
        const size_t capacity_;
        const std::exception_ptr closed_error_;
        std::mutex mutex_;
        std::condition_variable changed_;
        std::map<unsigned, std::shared_ptr<Entry>> entries_;
        std::deque<std::pair<unsigned, std::shared_ptr<Entry>>> queue_;
        size_t waiters_ = 0;
        uint64_t generation_ = 0;
        bool accepting_ = true;
        bool active_ = false;
        bool stopping_ = false;
        std::thread worker_;
    };
}
