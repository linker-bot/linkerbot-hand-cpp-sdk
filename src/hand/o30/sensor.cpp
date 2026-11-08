#include "../unified/detail/runtime_access.hpp"
#include "linkerbot/hand/o30/sensor_manager.hpp"
#include <linkerbot/hand/errors.hpp>

#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <linkerbot/hand/o30/protocol.hpp>

namespace linkerbot::hand::o30 {
    namespace {
        std::vector<uint8_t> read_exact(Client &client, uint8_t si, uint8_t length, std::chrono::milliseconds timeout) {
            auto data = client.read(protocol::kMiSensorInfo, si, length, false, timeout);
            if (data.size() != length) {
                throw protocol::ProtocolError("short sensor metadata reply");
            }
            return data;
        }

        void validate_descriptor(const std::string &text, size_t expected) {
            if (text.empty()) {
                if (expected != 0) {
                    throw protocol::ProtocolError("sensor data has no descriptor");
                }
                return;
            }
            size_t total = 0, start = 0;
            while (start < text.size()) {
                const auto end = text.find(';', start);
                const auto field = text.substr(start, end == std::string::npos ? end : end - start);
                const auto a = field.find('_');
                const auto b = a == std::string::npos ? a : field.find('_', a + 1);
                if (a == std::string::npos || b == std::string::npos) {
                    throw protocol::ProtocolError("malformed tactile descriptor");
                }
                const auto semantic = field.substr(0, a);
                const std::array<std::string, 11> semantics{"Fn",  "Ft", "Fd", "Px", "FnS", "FtS",
                                                            "FdS", "x",  "y",  "z",  "xyz"};
                if (std::find(semantics.begin(), semantics.end(), semantic) == semantics.end()) {
                    throw protocol::ProtocolError("unknown tactile descriptor semantic");
                }
                const auto type = field.substr(a + 1, b - a - 1);
                const size_t width = type == "U8" || type == "I8"                      ? 1
                                     : type == "U16" || type == "I16"                  ? 2
                                     : type == "U32" || type == "I32" || type == "F32" ? 4
                                                                                       : 0;
                if (!width || b + 1 == field.size() || field[b + 1] == '0') {
                    throw protocol::ProtocolError("invalid tactile descriptor type/count");
                }
                size_t count = 0;
                for (size_t i = b + 1; i < field.size(); ++i) {
                    if (field[i] < '0' || field[i] > '9' || count > 767) {
                        throw protocol::ProtocolError("invalid tactile descriptor count");
                    }
                    count = count * 10 + static_cast<size_t>(field[i] - '0');
                }
                total += count * width * (semantic == "xyz" ? 3 : 1);
                if (total > 767) {
                    throw protocol::ProtocolError("tactile descriptor exceeds 767 bytes");
                }
                if (end == std::string::npos) {
                    break;
                }
                start = end + 1;
                if (start == text.size()) {
                    throw protocol::ProtocolError("empty tactile descriptor field");
                }
            }
            if (total != expected) {
                throw protocol::ProtocolError("tactile descriptor length does not match total_length");
            }
        }
    }

    SensorManager::SensorManager(Client &client) : client_(&client), relay_(client.runtime(), "force") {}

    SensorInfo SensorManager::get_info(std::chrono::milliseconds timeout_ms) {
        auto operation = client_->operation(timeout_ms);
        const std::lock_guard<std::recursive_mutex> lock(mutex_);
        const bool modern = client_->revision(timeout_ms) == protocol::Revision::kV004;
        SensorInfo info{};
        info.type = protocol::decode_ascii(read_exact(*client_, 0x00, 0x20, timeout_ms));
        info.unit = protocol::decode_ascii(read_exact(*client_, 0x20, 0x08, timeout_ms));
        const auto shape = read_exact(*client_, 0x28, modern ? 4 : 6, timeout_ms);
        std::copy(shape.begin(), shape.end(), info.shape_bytes.begin());
        if (modern) {
            info.data_range = shape[0];
            info.maximum_rows = shape[1];
            info.maximum_columns = shape[2];
            const auto descriptor_length = shape[3];
            if (descriptor_length > 64) {
                throw protocol::ProtocolError("sensor descriptor exceeds 64 bytes");
            }
            if (descriptor_length) {
                const auto bytes = read_exact(*client_, 0x2C, descriptor_length, timeout_ms);
                info.descriptor = protocol::decode_ascii(bytes);
                if (info.descriptor.size() != bytes.size()) {
                    throw protocol::ProtocolError("unexpected padding in tactile descriptor");
                }
            }
            const auto control = read_exact(*client_, 0x6C, 7, timeout_ms);
            info.total_length = protocol::read_u16_le(control.data());
            info.selected = control[2];
            info.rows = control[3];
            info.columns = control[4];
            if (info.total_length > 767 || control[5] > 1 || control[6] > 1 || info.rows > info.maximum_rows ||
                info.columns > info.maximum_columns || info.selected > 5 || (info.total_length && !info.selected)) {
                throw protocol::ProtocolError("invalid HOP 0.0.4 sensor metadata");
            }
            info.flags = static_cast<uint16_t>(control[5] | (control[6] << 8U));
            validate_descriptor(info.descriptor, info.total_length);
            if (info.type == "NO_SENSOR" && info.total_length) {
                throw protocol::ProtocolError("NO_SENSOR reports tactile data");
            }
        } else {
            const auto length = read_exact(*client_, 0x2B, 2, timeout_ms);
            info.total_length = protocol::read_u16_le(length.data());
            info.selected = read_exact(*client_, 0x2D, 1, timeout_ms)[0];
            const auto matrix = read_exact(*client_, 0x2E, 4, timeout_ms);
            info.rows = matrix[0];
            info.columns = matrix[1];
            info.flags = protocol::read_u16_le(matrix.data() + 2);
        }
        info.timestamp = client_->received_at();
        return info;
    }

    void SensorManager::select(uint8_t sensor_index, std::chrono::milliseconds timeout_ms) {
        auto operation = client_->operation(timeout_ms);
        const std::lock_guard<std::recursive_mutex> lock(mutex_);
        const bool modern = client_->revision(timeout_ms) == protocol::Revision::kV004;
        if (modern && (sensor_index < 1 || sensor_index > 5)) {
            throw ValidationError("HOP 0.0.4 sensor index must be in 1..5");
        }
        const auto ack = client_->write(protocol::kMiSensorInfo, modern ? 0x6E : 0x2D, {sensor_index}, timeout_ms);
        if (modern && ack != std::vector<uint8_t>{sensor_index}) {
            throw protocol::ProtocolError("sensor selection did not echo requested index");
        }
    }

    std::vector<uint8_t> SensorManager::read_bank(uint8_t channel, const SensorInfo &info, bool modern,
                                                  std::chrono::milliseconds timeout_ms) {
        auto operation = client_->operation(timeout_ms);
        if (!info.total_length || info.type == "NO_SENSOR") {
            throw protocol::ProtocolError("sensor has no data; skip MI 0x32-0x34");
        }
        if (!modern && info.total_length > 256) {
            throw protocol::ProtocolError("legacy sensor channel exceeds the 256-byte SI address space");
        }
        const auto mi = static_cast<uint8_t>(protocol::kMiSensorCh1 + channel - 1);
        const size_t start = !modern || channel == 1 ? 0 : channel == 2 ? 255 : 511;
        const size_t capacity = !modern || channel > 1 ? 256 : 255;
        if (start >= info.total_length) {
            throw protocol::ProtocolError("sensor channel is outside total_length");
        }
        const auto length = std::min(capacity, static_cast<size_t>(info.total_length) - start);
        std::vector<uint8_t> data;
        while (data.size() < length) {
            const auto count = static_cast<uint8_t>(std::min(protocol::kMaxHopPayload, length - data.size()));
            const auto part = client_->read(mi, static_cast<uint8_t>(data.size()), count, false, timeout_ms);
            if (part.size() != count) {
                throw protocol::ProtocolError("short tactile data reply");
            }
            data.insert(data.end(), part.begin(), part.end());
        }
        return data;
    }

    SensorChannelData SensorManager::read_channel(uint8_t channel, std::chrono::milliseconds timeout_ms) {
        auto operation = client_->operation(timeout_ms);
        if (channel < 1 || channel > 3) {
            throw ValidationError("sensor channel must be 1, 2 or 3");
        }
        const std::lock_guard<std::recursive_mutex> lock(mutex_);
        const auto info = get_info(timeout_ms);
        const bool modern = client_->revision(timeout_ms) == protocol::Revision::kV004;
        return SensorChannelData{channel, read_bank(channel, info, modern, timeout_ms), client_->received_at()};
    }

    SensorData SensorManager::read_data(std::chrono::milliseconds timeout_ms) {
        auto operation = client_->operation(timeout_ms);
        const std::lock_guard<std::recursive_mutex> lock(mutex_);
        auto data = read_selected(timeout_ms);
        if (!data.info.total_length || data.info.type == "NO_SENSOR") {
            throw protocol::ProtocolError("sensor has no data; skip MI 0x32-0x34");
        }
        relay_.push(data);
        return data;
    }

    SensorData SensorManager::read_selected(std::chrono::milliseconds timeout_ms) {
        SensorData data{};
        data.info = get_info(timeout_ms);
        const bool modern = client_->revision(timeout_ms) == protocol::Revision::kV004;
        if (!modern && data.info.total_length > 256) {
            throw protocol::ProtocolError("legacy sensor data exceeds the verified channel address space");
        }
        for (uint8_t channel = 1; data.info.total_length && data.info.type != "NO_SENSOR" && channel <= 3; ++channel) {
            auto part = read_bank(channel, data.info, modern, timeout_ms);
            data.payload.insert(data.payload.end(), part.begin(), part.end());
            if (data.payload.size() == data.info.total_length) {
                break;
            }
        }
        data.timestamp = client_->received_at();
        return data;
    }

    SensorData SensorManager::read_all_data(std::chrono::milliseconds timeout_ms) {
        auto operation = client_->operation(timeout_ms);
        const std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (client_->revision(timeout_ms) != protocol::Revision::kV004) {
            throw UnsupportedOperationError("unified O30 tactile fields require HOP 0.0.4");
        }
        SensorData result;
        // Python response windows pass an unbounded composite budget. Saturate
        // its conversion instead of adding the rounded duration to a later now.
        const auto deadline = unified_detail::RuntimeAccess::uses_response_windows(*client_->runtime())
                                  ? runtime::deadline_after(timeout_ms)
                                  : std::chrono::steady_clock::now() + timeout_ms;
        const auto remaining = [&]() {
            const auto value =
                std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
            if (value.count() <= 0) {
                throw TimeoutError("O30 tactile total timeout expired");
            }
            return value;
        };
        for (uint8_t index = 1; index <= 5; ++index) {
            select(index, remaining());
            auto reading = read_selected(remaining());
            if (reading.info.selected != index) {
                throw protocol::ProtocolError("tactile sensor identity changed during read");
            }
            if (!reading.payload.empty()) {
                static_cast<void>(decode_tactile_fields({reading.info, reading.payload, reading.timestamp}));
            }
            if (index == 1) {
                result.info = reading.info;
            }
            result.timestamp = reading.timestamp;
            result.payload.insert(result.payload.end(), reading.payload.begin(), reading.payload.end());
            result.sensors.push_back({std::move(reading.info), std::move(reading.payload), reading.timestamp});
        }
        if (result.payload.empty()) {
            throw UnsupportedOperationError("O30 reports no available fingertip sensors");
        }
        // Publish only a complete validated round, never intermediate fingers.
        relay_.push(result);
        return result;
    }

    std::optional<SensorData> SensorManager::get_snapshot() const { return relay_.snapshot(); }

    void SensorManager::set_event_sink(std::function<void(const SensorData &)> sink) {
        relay_.set_sink(std::move(sink));
    }
} // namespace linkerbot::hand::o30
