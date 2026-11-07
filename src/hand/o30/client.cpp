#include <linkerbot/can/errors.hpp>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/o30/client.hpp>
#include <linkerbot/hand/o30/transaction.hpp>
#include <stdexcept>
#include <string>
#include <utility>

namespace linkerbot::hand::o30 {
    namespace {
        void validate_id(uint32_t id) {
            if (id > 0x3ff) {
                throw ValidationError("O30 request ID must leave an 11-bit response ID");
            }
        }
        auto borrow(can::CANDispatcher &dispatcher, uint32_t id) {
            validate_id(id);
            return runtime::DeviceRuntime::borrow(dispatcher, "o30/" + std::to_string(id),
                                                  {{protocol::expected_response_id(id), false}});
        }
    }
    Client::Client(can::CANDispatcher &dispatcher, uint32_t id) : Client(borrow(dispatcher, id), id) {}
    Client::Client(std::shared_ptr<runtime::DeviceRuntime> runtime, uint32_t id)
        : runtime_(std::move(runtime)), request_id_(id), response_id_(protocol::expected_response_id(id)) {
        validate_id(id);
        if (!runtime_) {
            throw ValidationError("O30 Client requires a runtime");
        }
    }
    Client::~Client() {
        try {
            close();
        } catch (...) {
            try {
                runtime_->quiesce_for_destruction();
                calls_.drain();
                // Destruction is nonthrowing; explicit close/stop remains the error-reporting path.
                // NOLINTNEXTLINE(bugprone-empty-catch)
            } catch (...) {
            }
        }
    }
    void Client::close() {
        runtime_->close();
        calls_.drain();
    }
    void Client::set_request_id(uint32_t id) {
        validate_id(id);
        auto call = calls_.hold();
        auto operation = runtime_->operation(runtime::Deadline::max());
        runtime_->set_routes({{protocol::expected_response_id(id), false}});
        request_id_ = id;
        response_id_ = protocol::expected_response_id(id);
        revision_.reset();
    }
    void Client::change_request_id(uint16_t id, std::chrono::milliseconds timeout) {
        validate_id(id);
        auto operation = this->operation(timeout);
        std::vector<uint8_t> bytes;
        protocol::append_u16_le(bytes, id);
        (void)transact(protocol::kMiConfig, 0, 2, bytes, false, timeout, id);
        revision_.reset();
    }
    protocol::Revision Client::revision(std::chrono::milliseconds timeout) {
        auto operation = this->operation(timeout);
        const auto generation = runtime_->runtime_health().device_generation;
        if (generation != revision_generation_) {
            revision_.reset();
        }
        if (!revision_) {
            const auto bytes = read(protocol::kMiProductInfo, 0x61, 8, false, timeout);
            if (bytes.size() != 8) {
                throw protocol::ProtocolError("short HOP protocol version reply");
            }
            auto text = protocol::decode_ascii(bytes);
            if (!text.empty() && (text.front() == 'v' || text.front() == 'V')) {
                text.erase(0, 1);
            }
            if (text == "0.0.3") {
                revision_ = protocol::Revision::kV003;
            } else if (text == "0.0.4") {
                revision_ = protocol::Revision::kV004;
            } else {
                throw protocol::ProtocolError("unsupported HOP version: " + text);
            }
            revision_generation_ = generation;
        }
        return *revision_;
    }
    std::vector<uint8_t> Client::read(uint8_t mi, uint8_t si, uint8_t edl, bool rts,
                                      std::chrono::milliseconds timeout) {
        return transact(mi, si, edl, {}, rts, timeout);
    }
    std::vector<uint8_t> Client::write(uint8_t mi, uint8_t si, const std::vector<uint8_t> &payload,
                                       std::chrono::milliseconds timeout) {
        auto operation = this->operation(timeout);
        if (payload.empty() || payload.size() > protocol::kMaxHopPayload ||
            static_cast<size_t>(si) + payload.size() > 256 || mi > 0x7f) {
            throw ValidationError("invalid O30 write MI, SI or payload length");
        }
        const bool rts = mi == protocol::kMiCurrent && revision(timeout) == protocol::Revision::kV004;
        return transact(mi, si, static_cast<uint8_t>(payload.size()), payload, rts, timeout);
    }
    void Client::write_mounted(uint8_t mi, const std::array<uint8_t, kSlotCount> &slots,
                               std::chrono::milliseconds timeout) {
        auto operation = this->operation(timeout);
        for (const auto &segment : kSafeWriteSegments) {
            const std::vector<uint8_t> slice(slots.begin() + segment.si, slots.begin() + segment.si + segment.edl);
            (void)write(mi, segment.si, slice, timeout);
        }
    }
    std::vector<uint8_t> Client::transact(uint8_t mi, uint8_t si, uint8_t edl, const std::vector<uint8_t> &payload,
                                          bool rts, std::chrono::milliseconds timeout,
                                          std::optional<uint32_t> next_id) {
        const auto deadline = runtime::deadline_after(timeout);
        if (runtime_->runtime_health().state == runtime::DeviceState::closed) {
            throw ClosedError("O30 client is closed");
        }
        auto call = calls_.hold();
        auto operation = runtime_->operation(deadline);
        const auto old_response = response_id_.load();
        if (next_id && protocol::expected_response_id(*next_id) != old_response) {
            runtime_->set_routes({{old_response, false}, {protocol::expected_response_id(*next_id), false}});
        }
        try {
            std::vector<uint8_t> bytes;
            try {
                bytes = runtime_->query<std::vector<uint8_t>>(
                    protocol::transaction(request_id_.load(), mi, si, edl, payload, rts, next_id), deadline);
            } catch (const TimeoutError &error) {
                if (dynamic_cast<const ResponseTimeoutError *>(&error)) {
                    throw;
                }
                throw TimeoutError("No O30 response received within " + std::to_string(timeout.count()) +
                                       "ms: " + error.what(),
                                   error.context(), std::current_exception());
            }
            if (next_id) {
                if (bytes.size() != 2) {
                    throw protocol::ProtocolError("short standard-ID write acknowledgement");
                }
                request_id_ = *next_id;
                response_id_ = protocol::expected_response_id(*next_id);
                runtime_->set_routes({{response_id_.load(), false}});
            }
            return bytes;
        } catch (...) {
            // Cancellation/close can make set_routes fail; retain the original error.
            const auto original = std::current_exception();
            if (next_id) {
                try {
                    runtime_->set_routes({{old_response, false}});
                } catch (...) {
                    std::rethrow_exception(original);
                }
            }
            throw;
        }
    }
} // namespace linkerbot::hand::o30
