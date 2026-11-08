#pragma once
#include "runtime_access.hpp"
#include <condition_variable>
#include <linkerbot/hand/api/device.hpp>
#include <map>
#include <mutex>
#include <set>

namespace linkerbot::hand::unified_detail {
    void read_python_scalar(runtime::DeviceRuntime &, const HandDeviceConfig &, TelemetrySemantic);

    // Bounded per-device publication/poll intent state. It never sends frames.
    class PythonState {
    public:
        struct View {
            std::map<std::string, runtime::Sample> samples;
            std::map<TelemetrySemantic, SampleMeta> metadata;
            uint64_t sequence = 0;
        };
        explicit PythonState(const HandDeviceConfig &config);
        void publish(const std::string &domain, const runtime::Sample &sample);
        View view(bool include_feedback = true) const;
        void wait(uint64_t sequence, runtime::Deadline deadline, bool include_feedback,
                  const std::function<bool()> &generation_valid = {}) const;
        void stop();
        void reset();
        void resume();
        void initialized();
        DeviceCapabilities capabilities(DeviceCapabilities value) const;
        bool disabled(TelemetrySemantic semantic) const;
        void read_succeeded(TelemetrySemantic semantic, bool polling);
        void read_failed(TelemetrySemantic semantic, bool sent, const HandError &error, bool polling);
        bool replace(const PollingPlan &plan);
        void stop_polling();
        std::optional<runtime::Deadline> next_poll() const;
        std::optional<TelemetrySemantic> take_poll();
        void project_health(HandHealth &health) const;
        std::string domain(TelemetrySemantic semantic) const;
        void project(HandSnapshot &snapshot, const View &view, const DeviceCapabilities &caps) const;

    private:
        struct Poll {
            PollingChannel channel;
            runtime::Deadline next;
            runtime::Deadline started;
            uint64_t reads = 0;
            uint64_t feedback = 0;
            uint64_t commands = 0;
            uint64_t missed = 0;
            uint64_t timeouts = 0;
            bool enabled = true;
        };
        bool optional(TelemetrySemantic semantic) const;
        void disable_locked(TelemetrySemantic semantic);
        void visible_locked(bool command_feedback);
        HandDeviceConfig config_;
        std::map<std::string, TelemetrySemantic> domains_;
        mutable std::mutex mutex_;
        mutable std::condition_variable changed_;
        View current_;
        View non_command_;
        std::map<TelemetrySemantic, uint64_t> sequences_;
        std::set<TelemetrySemantic> verified_;
        std::set<TelemetrySemantic> disabled_;
        std::map<TelemetrySemantic, unsigned> failures_;
        std::map<TelemetrySemantic, Poll> polls_;
        std::optional<PollingPlan> plan_;
        uint64_t identity_sequence_ = 0;
        bool stopped_ = false;
        bool ready_ = false;
    };
}
