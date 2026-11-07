#pragma once

#include "backend.hpp"

#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <thread>

namespace linkerbot::hand::unified_detail {
    CommandFamily command_family(CommandSemantic semantic);
    const char *command_family_name(CommandFamily family);

    // One executor per device, plus a queue deadline watcher. The watcher never
    // performs I/O; it expires pending work even while the executor is in a driver.
    class CommandScheduler {
    public:
        using Clock = std::chrono::steady_clock;
        using Now = std::function<Clock::time_point()>;
        using Execute = std::function<CommandHandle(const CommandRequest &)>;
        CommandScheduler(double rate_hz, size_t capacity, Now now = {}, bool python_behavior = false);
        ~CommandScheduler();
        CommandScheduler(const CommandScheduler &) = delete;
        CommandScheduler &operator=(const CommandScheduler &) = delete;

        // The facade holds its state lock across admission/invalidation, making
        // Ready, generation and queue installation one linearization point.
        CommandHandle accept(CommandRequest request, AckSupport support, ErrorContext context, Execute execute);
        void invalidate(std::exception_ptr error);
        std::vector<CommandFamilyMetrics> metrics() const;
        // Internal test seam: the caller advances its monotonic clock before
        // notifying. Now must be monotonic, thread-safe and nonthrowing.
        // Production retains steady_clock and timed CV waits. This seam controls
        // queue expiry/throttling, not native protocol response clocks.
        void notify_clock_advanced();

    private:
        struct ConstructorTag {};
        struct ConstructorArguments {
            double rate_hz;
            size_t capacity;
            Now now;
            bool python_behavior;
        };
        CommandScheduler(ConstructorTag, ConstructorArguments arguments);

        struct Job {
            CommandRequest request;
            CommandHandle handle;
            ErrorContext context;
            Clock::time_point accepted;
            Clock::time_point deadline;
            Execute execute;
            runtime::CancellationSource cancellation;
        };
        struct Metrics {
            CommandFamilyMetrics value;
            std::deque<HealthDuration> queue_times;
            std::deque<HealthDuration> execution_times;
        };
        static bool latest(CommandFamily family);
        void fail(Job &job, std::exception_ptr error);
        void expire(Clock::time_point now);
        Clock::time_point next_deadline() const;
        size_t pending_size() const;
        void removed(Job &job, bool dropped);
        void run();
        void watch_deadlines();

        const Clock::duration interval_;
        const size_t capacity_;
        const bool manual_clock_;
        const bool python_behavior_;
        const Now now_;
        mutable std::mutex mutex_;
        std::condition_variable changed_;
        std::deque<Job> fifo_;
        std::map<CommandFamily, Job> latest_;
        std::map<CommandFamily, Metrics> metrics_;
        std::optional<Job> active_;
        uint64_t next_id_ = 1;
        bool stop_ = false;
        uint64_t invalidation_ = 0;
        std::optional<runtime::CancellationSource> preparing_;
        Clock::time_point next_start_ = Clock::time_point::min();
        std::thread worker_;
        std::thread watcher_;
    };
}
