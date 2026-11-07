#include <atomic>
#include <linkerbot/can/errors.hpp>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/l30/client.hpp>
#include <linkerbot/hand/l30/protocol_backend.hpp>

namespace linkerbot::hand::l30 {
    namespace {
        std::shared_ptr<runtime::DeviceRuntime> acquire(std::shared_ptr<transport::Session> session, uint8_t node,
                                                        uint8_t host) {
            if (node < protocol::kNodeIdMin || node > protocol::kNodeIdMax || host > protocol::kHostIdMax) {
                throw ValidationError("L30 node_id must be 1..31 and host_id 0..31");
            }
            // Client's public raw-command interface accepts every legal parent/subcommand.
            // Reserve those exact response IDs, plus every priority of unsolicited reports.
            std::vector<transport::RouteKey> routes;
            for (unsigned parent = 0; parent < 16; ++parent) {
                for (unsigned sub = 0; sub < 256; ++sub) {
                    for (unsigned access = 0; access < 2; ++access) {
                        routes.push_back(
                            {protocol::FrameId{0, static_cast<uint8_t>(access), static_cast<uint8_t>(parent),
                                               static_cast<uint8_t>(sub), host, node}
                                 .to_int(),
                             true});
                    }
                }
            }
            for (unsigned priority = 1; priority < 8; ++priority) {
                for (unsigned sub = 0; sub < 256; ++sub) {
                    routes.push_back({protocol::FrameId{static_cast<uint8_t>(priority), 0, protocol::kParentPeriodic,
                                                        static_cast<uint8_t>(sub), host, node}
                                          .to_int(),
                                      true});
                }
            }
            return runtime::DeviceRuntime::acquire(
                std::move(session), "l30:" + std::to_string(node) + ":" + std::to_string(host), std::move(routes));
        }
        std::atomic<uint64_t> next_observer{0};
        struct ReportContext {
            const Client *client;
            const runtime::Sample *sample;
        };
        thread_local ReportContext report_context{};
    }
    Client::Client(can::CANDispatcher &dispatcher, uint8_t node, uint8_t host)
        : Client(transport::Session::borrow(dispatcher), node, host) {}
    Client::Client(std::shared_ptr<transport::Session> session, uint8_t node, uint8_t host)
        : Client(acquire(std::move(session), node, host), node, host) {}
    Client::Client(std::shared_ptr<runtime::DeviceRuntime> core, uint8_t node, uint8_t host)
        : core_(std::move(core)), node_id_(node), host_id_(host) {
        if (!core_ || acquire(core_->session(), node, host) != core_) {
            throw ValidationError("L30 runtime binding does not match node/host");
        }
        report_domain_ = "l30.reports." + std::to_string(++next_observer);
        observer_ = core_->observe(report_domain_,
                                   [node, host](const auto &m) { return protocol::decode_report(m, node, host); });
    }
    Client::~Client() {
        try {
            close();
            if (can::CANDispatcher::is_any_receive_thread() || core_->is_event_thread()) {
                core_->quiesce_for_destruction();
                calls_.drain();
            }
            // Destruction is nonthrowing; explicit close/stop remains the error-reporting path.
            // NOLINTNEXTLINE(bugprone-empty-catch)
        } catch (...) {
        }
    }
    void Client::throw_if_closed() const {
        if (closed_ || (stopping_ && shutdown_thread_ != std::this_thread::get_id())) {
            throw ClosedError("L30 client is closed");
        }
    }
    std::shared_ptr<void> Client::enter() {
        const auto thread = std::this_thread::get_id();
        std::unique_lock<std::mutex> lock(mutex_);
        throw_if_closed();
        if (cancelled_threads_.count(thread)) {
            throw LifecycleError("L30 request cancelled");
        }
        auto lifetime = calls_.hold();
        ++scopes_[thread].depth;
        lock.unlock();
        return std::shared_ptr<void>(this, [this, thread, lifetime = std::move(lifetime)](void *) {
            std::lock_guard<std::mutex> guard(mutex_);
            const auto it = scopes_.find(thread);
            if (it != scopes_.end() && !--it->second.depth) {
                scopes_.erase(it);
            }
        });
    }
    runtime::CancellationToken Client::token() {
        std::lock_guard<std::mutex> lock(mutex_);
        throw_if_closed();
        return scopes_.at(std::this_thread::get_id()).cancel.token();
    }
    Client::Operation::Operation(Client &client, int kind, runtime::Deadline deadline) : deadline_(deadline) {
        if (can::CANDispatcher::is_any_receive_thread()) {
            throw LifecycleError("L30 synchronous operation from receive callback");
        }
        auto call = client.enter();
        operation_ =
            kind == 2 ? client.core_->operation(deadline, client.token())
                      : client.core_->operation(kind == 1 ? "l30.tactile" : "l30.ordinary", deadline, client.token());
        deadline_ = operation_.deadline();
        operation_.retain_call(std::move(call));
    }
    Client::Operation::~Operation() = default;
    std::chrono::milliseconds Client::Operation::remaining() const { return runtime::remaining(deadline_); }
    Client::Operation Client::operation(std::chrono::milliseconds timeout, bool tactile_only) {
        return Operation(*this, tactile_only ? 1 : 2, runtime::deadline_after(timeout));
    }
    void Client::cancel_thread(std::thread::id thread) {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_threads_.insert(thread);
        const auto it = scopes_.find(thread);
        if (it != scopes_.end()) {
            it->second.cancel.cancel();
        }
    }
    void Client::resume_thread(std::thread::id thread) {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_threads_.erase(thread); // Existing cancelled scopes never revive.
    }
    void Client::begin_shutdown() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_) {
            return;
        }
        stopping_ = true;
        shutdown_thread_ = std::this_thread::get_id();
        for (auto &[thread, scope] : scopes_) {
            if (thread != shutdown_thread_) {
                scope.cancel.cancel();
            }
        }
        core_->cancel_all();
    }
    void Client::close() {
        std::vector<HandlerId> handlers;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
            for (auto &[thread, scope] : scopes_) {
                (void)thread;
                scope.cancel.cancel();
            }
            for (const auto &[id, subcmd] : handlers_) {
                (void)subcmd;
                handlers.push_back(id);
            }
        }
        core_->cancel_all();
        for (auto id : handlers) {
            core_->unsubscribe(id);
        }
        core_->unobserve(observer_);
        if (can::CANDispatcher::is_any_receive_thread() || core_->is_event_thread()) {
            return;
        }
        core_->close();
        calls_.drain();
    }
    void Client::remember(runtime::Sample sample) {
        std::lock_guard<std::mutex> lock(mutex_);
        scopes_.at(std::this_thread::get_id()).sample = std::move(sample);
    }
    void Client::publish(const std::string &domain, std::any value) {
        runtime::Sample sample;
        if (report_context.client == this) {
            sample = *report_context.sample;
        } else {
            std::lock_guard<std::mutex> lock(mutex_);
            throw_if_closed();
            const auto &saved = scopes_.at(std::this_thread::get_id()).sample;
            if (!saved) {
                throw LifecycleError("L30 publication requires completed I/O");
            }
            sample = *saved;
        }
        core_->publish(domain, std::move(value), std::move(sample));
    }
    void Client::send_no_ack(uint8_t parent, uint8_t subcmd, const std::vector<uint8_t> &payload, uint8_t bytes) {
        const Operation op(*this, 0, runtime::Deadline::max());
        remember(core_->execute(protocol::request_plan(protocol::request_frame(node_id_, host_id_, parent, subcmd,
                                                                               protocol::kAccessWrite, payload, bytes),
                                                       false),
                                op.deadline(), token()));
    }
    std::vector<uint8_t> Client::request_ack(uint8_t parent, uint8_t subcmd, const std::vector<uint8_t> &payload,
                                             uint8_t bytes, std::chrono::milliseconds timeout) {
        return request(parent, subcmd, protocol::kAccessWrite, payload, bytes, timeout);
    }
    std::vector<uint8_t> Client::read(uint8_t parent, uint8_t subcmd, std::chrono::milliseconds timeout) {
        return request(parent, subcmd, protocol::kAccessRead, protocol::empty_request_payload(),
                       protocol::kEmptyRequestBytes, timeout);
    }
    std::vector<uint8_t> Client::request(uint8_t parent, uint8_t subcmd, uint8_t access,
                                         const std::vector<uint8_t> &payload, uint8_t bytes,
                                         std::chrono::milliseconds timeout) {
        const auto deadline = runtime::deadline_after(timeout);
        try {
            const Operation op(*this, 0, deadline);
            auto sample = core_->execute(protocol::request_plan(protocol::request_frame(
                                             node_id_, host_id_, parent, subcmd, access, payload, bytes)),
                                         deadline, token());
            remember(sample);
            return std::any_cast<std::vector<uint8_t>>(std::move(sample.value));
        } catch (const DeviceNotReadyError &e) {
            throw QuarantinedError(std::string("L30 response ID quarantined; ") + e.what(), e.context(),
                                   std::current_exception());
        } catch (const TimeoutError &e) {
            if (dynamic_cast<const ResponseTimeoutError *>(&e)) {
                throw;
            }
            throw TimeoutError("No L30 response received within " + std::to_string(timeout.count()) + "ms: " + e.what(),
                               e.context(), std::current_exception());
        }
    }
    std::array<uint8_t, protocol::kTactileDataLength> Client::read_tactile(uint8_t subcmd,
                                                                           std::chrono::milliseconds timeout) {
        const auto deadline = runtime::deadline_after(timeout);
        try {
            const Operation op(*this, 1, deadline);
            auto sample = core_->execute(protocol::tactile_plan(node_id_, host_id_, subcmd), deadline, token());
            remember(sample);
            return std::any_cast<std::array<uint8_t, protocol::kTactileDataLength>>(std::move(sample.value));
        } catch (const DeviceNotReadyError &e) {
            throw QuarantinedError(std::string("L30 tactile response ID quarantined; ") + e.what(), e.context(),
                                   std::current_exception());
        } catch (const TimeoutError &e) {
            if (dynamic_cast<const ResponseTimeoutError *>(&e)) {
                throw;
            }
            throw TimeoutError("No L30 tactile response received within " + std::to_string(timeout.count()) +
                                   "ms: " + e.what(),
                               e.context(), std::current_exception());
        }
    }
    Client::HandlerId Client::add_report_handler(uint8_t subcmd, ReportHandler handler) {
        std::unique_lock<std::mutex> lock(mutex_);
        throw_if_closed();
        auto id =
            core_->subscribe(report_domain_, [this, subcmd, handler = std::move(handler)](const runtime::Sample &s) {
                const auto &report = std::any_cast<const protocol::Report &>(s.value);
                if (report.subcmd == subcmd && handler) {
                    const auto previous = report_context;
                    struct Restore {
                        ReportContext previous;
                        ~Restore() { report_context = previous; }
                    } restore{previous};
                    report_context = {this, &s};
                    handler(report.payload);
                }
            });
        try {
            handlers_.emplace(id, subcmd);
        } catch (...) {
            lock.unlock();
            core_->unsubscribe(id);
            throw;
        }
        return id;
    }
    void Client::remove_report_handler(uint8_t subcmd, HandlerId id) {
        struct HandlerBinding {
            HandlerId handler;
            uint8_t subcommand;
        };
        const auto binding = HandlerBinding{id, subcmd};
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto it = handlers_.find(binding.handler);
            if (it == handlers_.end() || it->second != binding.subcommand) {
                return;
            }
        }
        if (core_->unsubscribe(binding.handler)) {
            std::lock_guard<std::mutex> lock(mutex_);
            handlers_.erase(binding.handler);
        }
    }
} // namespace linkerbot::hand::l30
