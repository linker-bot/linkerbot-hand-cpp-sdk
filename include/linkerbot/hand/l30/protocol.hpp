#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <linkerbot/can/message.hpp>
#include <linkerbot/hand/errors.hpp>
#include <linkerbot/hand/l30/joints.hpp>

// L30 CAN FD extended-frame protocol (v6).
//
// Unlike the classic-CAN hands, L30 does not put a command code in BYTE0. The
// command lives in the 29-bit arbitration ID, and BYTE0/BYTE1 carry a payload
// length and a transaction-control byte. Multi-byte values are big-endian.
namespace linkerbot::hand::l30::protocol {

    // ---- Arbitration ID bit fields (section 3.1) ----------------------------
    //
    // BIT28:26 Priority | BIT25 Access | BIT24:21 ParentCmd
    // BIT20:13 SubCmd   | BIT12:8 DstID | BIT7:3 SrcID | BIT2:0 Reserved(0)

    inline constexpr uint8_t kPriorityControl = 0;

    inline constexpr uint8_t kAccessRead = 0;
    inline constexpr uint8_t kAccessWrite = 1;

    inline constexpr uint8_t kParentControl = 0x1;
    inline constexpr uint8_t kParentTactile = 0x2;
    inline constexpr uint8_t kParentConfig = 0x3;
    inline constexpr uint8_t kParentPeriodic = 0x4;
    inline constexpr uint8_t kParentQuery = 0x5;

    // Control 0x1/0x02 sets current limits; query 0x5/0x02 reads current.
    inline constexpr uint8_t kSubPosition = 0x01;
    inline constexpr uint8_t kSubCurrentLimit = 0x02;
    inline constexpr uint8_t kSubCurrent = 0x02;
    inline constexpr uint8_t kSubSpeed = 0x03;
    inline constexpr uint8_t kSubTemperature = 0x04;
    inline constexpr uint8_t kSubFault = 0x05;
    inline constexpr uint8_t kSubEnable = 0x07;
    inline constexpr uint8_t kSubDisable = 0x08;

    // Config (parent 0x3) subcommands.
    inline constexpr uint8_t kSubConfigUnlock = 0x01;
    inline constexpr uint8_t kSubDeviceInfo = 0x02;
    inline constexpr uint8_t kSubProductCode = 0x03;
    inline constexpr uint8_t kSubNodeId = 0x04;
    inline constexpr uint8_t kSubCalibrate = 0x05;
    inline constexpr uint8_t kSubHandType = 0x06;

    // Tactile (parent 0x2) per-finger subcommands.
    inline constexpr uint8_t kSubTactileThumb = 0x01;
    inline constexpr uint8_t kSubTactileIndex = 0x02;
    inline constexpr uint8_t kSubTactileMiddle = 0x03;
    inline constexpr uint8_t kSubTactileRing = 0x04;
    inline constexpr uint8_t kSubTactilePinky = 0x05;

    // ---- Payload framing (section 2.2 / 4.1) -------------------------------

    inline constexpr size_t kLengthByteIndex = 0;
    inline constexpr size_t kTransactionByteIndex = 1;
    inline constexpr size_t kStatusByteIndex = 2;
    inline constexpr size_t kResponseDataOffset = 3;

    inline constexpr uint8_t kSingleFrameTransaction = 0x00;
    inline constexpr uint8_t kEmptyPayloadLength = 0x00;
    inline constexpr uint8_t kStatusOk = 0x00;

    // Wire byte counts each request occupies (the comm layer takes a byte count
    // and the SocketCAN backend writes it straight into canfd_frame.len, so the
    // padding to a legal CAN FD length has to happen here).
    inline constexpr uint8_t kEmptyRequestBytes = 2;
    inline constexpr uint8_t kAckBytes = 3;
    inline constexpr uint8_t kVectorPayloadBytes = 34; // 17 joints x 2 bytes
    inline constexpr uint8_t kVectorWireBytes = 48;    // DLC 0x0E
    inline constexpr uint8_t kU8VectorPayloadBytes = 17;
    inline constexpr uint8_t kU8VectorWireBytes = 20; // DLC 0x0B
    // Section 10.1 writes this as "DLC=0x10", meaning 16 bytes decimal; the
    // actual DLC nibble for 16 wire bytes is 0x0A.
    inline constexpr uint8_t kPeriodicConfigWireBytes = 16;

    // ---- Periodic reporting (section 10) ----------------------------------

    inline constexpr uint8_t kPeriodicConfigPayloadBytes = 9;
    inline constexpr uint32_t kPeriodicMinPeriodMs = 20;
    inline constexpr uint32_t kPeriodicMaxPeriodMs = 600000;
    inline constexpr uint32_t kPeriodicAllJointsMask = 0;
    inline constexpr uint32_t kJointMaskLimit = (1U << kJointCount) - 1U;

    // ---- Tactile multi-frame reassembly (section 8.2) ----------------------

    inline constexpr uint8_t kTactileFirstTransaction = 0x10;
    inline constexpr uint8_t kTactileSecondTransaction = 0x11;
    inline constexpr size_t kTactileFirstLength = 61;
    inline constexpr size_t kTactileSecondLength = 11;
    inline constexpr size_t kTactileDataLength = 72; // 12 rows x 6 columns
    inline constexpr size_t kTactileRows = 12;
    inline constexpr size_t kTactileColumns = 6;

    // ---- Addressing -------------------------------------------------------

    inline constexpr uint8_t kNodeIdMin = 1;
    inline constexpr uint8_t kNodeIdMax = 31;
    inline constexpr uint8_t kHostIdMin = 0;
    inline constexpr uint8_t kHostIdMax = 31;

    inline constexpr size_t kDeviceInfoBytes = 18;

    // Raised when a response violates the framing rules or the device reports a
    // non-zero status. Callers that need to react to a specific device state
    // (0x20 not unlocked, 0x23 operation not allowed, ...) read status().
    class ProtocolError : public ::linkerbot::hand::ProtocolError {
    public:
        explicit ProtocolError(const std::string &message, uint8_t status = kStatusOk)
            : ::linkerbot::hand::ProtocolError(message), status_(status) {}

        // kStatusOk when the failure was a framing problem rather than a device
        // status report.
        [[nodiscard]] uint8_t status() const { return status_; }

    private:
        uint8_t status_;
    };

    // Decoded fields of an L30 29-bit arbitration ID. A response keeps the
    // request's Access bit and swaps DstID/SrcID, which is why write ACKs still
    // carry Access=1.
    struct FrameId {
        uint8_t priority = kPriorityControl;
        uint8_t access = kAccessRead;
        uint8_t parent = 0;
        uint8_t subcmd = 0;
        uint8_t dst_id = 0;
        uint8_t src_id = 0;

        [[nodiscard]] uint32_t to_int() const;

        // Expected reply for this request: IDs swapped, Access preserved.
        [[nodiscard]] FrameId response() const;
    };

    [[nodiscard]] uint32_t build_can_id(const FrameId &fields);

    // Throws std::invalid_argument if the ID exceeds 29 bits or uses the
    // reserved low three bits.
    [[nodiscard]] FrameId parse_can_id(uint32_t arbitration_id);

    [[nodiscard]] uint32_t expected_response_id(uint32_t request_id);

    // Builds one extended CAN FD frame, zero-padding the payload out to
    // wire_bytes so the device sees the DLC the protocol specifies.
    [[nodiscard]] can::Message build_message(const FrameId &fields, const std::vector<uint8_t> &payload,
                                             uint8_t wire_bytes);

    // ---- Payload encoders --------------------------------------------------

    [[nodiscard]] std::vector<uint8_t> empty_request_payload();

    [[nodiscard]] std::vector<uint8_t> ack_payload(uint8_t status = kStatusOk);

    [[nodiscard]] std::vector<uint8_t> encode_i16_vector(const std::array<int16_t, kJointCount> &values);

    [[nodiscard]] std::vector<uint8_t> encode_u16_vector(const std::array<uint16_t, kJointCount> &values);

    [[nodiscard]] std::vector<uint8_t> encode_periodic_config(bool enabled, uint32_t period_ms, uint32_t joint_mask);

    // ---- Response decoders -------------------------------------------------
    //
    // Every decoder validates BYTE0 (declared length), BYTE1 (transaction) and
    // BYTE2 (status) before touching data, and throws ProtocolError on a
    // mismatch or a non-zero device status.

    [[nodiscard]] std::array<int16_t, kJointCount> decode_i16_response(const uint8_t *data, size_t len);

    [[nodiscard]] std::array<uint8_t, kJointCount> decode_u8_response(const uint8_t *data, size_t len);

    // Validates the status byte of an ACK that carries no payload.
    void check_response_status(const uint8_t *data, size_t len);

    // Returns the useful payload of a status-checked response.
    [[nodiscard]] std::vector<uint8_t> decode_response_payload(const uint8_t *data, size_t len, size_t expected_length);

    // ASCII payload with trailing NULs stripped. Trusts BYTE0 for the length.
    [[nodiscard]] std::string decode_ascii_response(const uint8_t *data, size_t len);

    // Periodic reports are unsolicited and carry no status byte, so data starts
    // at BYTE2 rather than BYTE3.
    [[nodiscard]] std::vector<int16_t> decode_periodic_i16_report(const uint8_t *data, size_t len, uint32_t joint_mask);

    [[nodiscard]] std::vector<uint8_t> decode_periodic_u8_report(const uint8_t *data, size_t len, uint32_t joint_mask);

    // Slices 61 + 11 useful bytes out of the two tactile frames, ignoring the
    // CAN FD padding each one carries.
    [[nodiscard]] std::array<uint8_t, kTactileDataLength> assemble_tactile_payload(const std::vector<uint8_t> &first,
                                                                                   const std::vector<uint8_t> &second);

    [[nodiscard]] std::string_view status_message(uint8_t status);

    // Number of joints a report carries: popcount of the mask, or all 17 when
    // the mask is zero.
    [[nodiscard]] size_t reported_joint_count(uint32_t joint_mask);

} // namespace linkerbot::hand::l30::protocol
