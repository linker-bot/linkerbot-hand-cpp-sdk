#pragma once
#include <algorithm>
#include <array>
#include <linkerbot/hand/capabilities.hpp>
#include <linkerbot/hand/protocol/transaction.hpp>
#include <optional>
#include <string>

namespace linkerbot::hand::unified_detail {
    // Python common/classic_can.py serial assembly. The original native
    // Manager's framing rules remain available on its default path.
    inline std::function<protocol::Progress(const can::Message &)> python_serial(uint32_t id, Model model) {
        return [id, model, fragments = std::array<std::optional<std::string>, 4>{},
                terminal = std::optional<size_t>{}](const can::Message &message) mutable {
            if (message.id != id || message.is_extended || message.is_fd || message.is_brs || message.dlc == 0 ||
                message.data[0] != 0xC0) {
                return protocol::Progress::unmatched();
            }
            if (message.dlc < 3 || message.dlc > 8) {
                throw ProtocolError("serial response must include marker and 1..6 bytes");
            }
            const auto marker = message.data[1];
            const size_t slot = marker < 4 ? marker : marker <= 18 && marker % 6 == 0 ? marker / 6 : 4;
            if (slot == 4) {
                throw ProtocolError("serial fragment marker is outside the supported product code");
            }
            const std::string payload(message.data.begin() + 2, message.data.begin() + message.dlc);
            if (fragments[slot] && *fragments[slot] != payload) {
                throw ProtocolError("serial fragment conflicts with a duplicate");
            }
            fragments[slot] = payload;
            if (payload.size() < 6 || payload.find('\0') != std::string::npos) {
                terminal = terminal ? std::min(*terminal, slot) : slot;
            }
            bool complete =
                std::all_of(fragments.begin(), fragments.end(), [](const auto &part) { return part.has_value(); });
            if (!complete && terminal) {
                complete = true;
                for (size_t i = 0; i < fragments.size(); ++i) {
                    if (fragments[i].has_value() != (i <= *terminal)) {
                        complete = false;
                    }
                }
            }
            std::string code;
            for (const auto &fragment : fragments) {
                if (fragment) {
                    code += *fragment;
                }
            }
            while (!code.empty() && code.back() == '\0') {
                code.pop_back();
            }
            if (!complete && fragments[0] && fragments[1] && fragments[2] && !fragments[3]) {
                const auto fields = static_cast<size_t>(std::count(code.begin(), code.end(), '-')) + 1;
                bool nonempty =
                    !code.empty() && code.front() != '-' && code.back() != '-' && code.find("--") == std::string::npos;
                if (model == Model::kL6) {
                    size_t begin = 0;
                    while (nonempty && begin < code.size()) {
                        const auto end = code.find('-', begin);
                        const auto field = code.substr(begin, end == std::string::npos ? end : end - begin);
                        nonempty = field.find_first_not_of(" \t\r\n") != std::string::npos;
                        begin = end == std::string::npos ? code.size() : end + 1;
                    }
                    complete = (fields == 6 || fields == 7) && nonempty;
                } else if (model == Model::kL20Lite) {
                    complete = code.size() == 18 && fields == 7 && nonempty && code[code.size() - 2] == '-';
                }
            }
            if (!complete) {
                return protocol::Progress::pending();
            }
            if (code.empty() ||
                std::any_of(code.begin(), code.end(), [](unsigned char value) { return value > 127; })) {
                throw ProtocolError("factory product code must be nonempty ASCII");
            }
            // Progress owns the moved string through std::any; the analyzer
            // does not model the type-erased owner's eventual destruction.
            // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
            return protocol::Progress::complete(std::move(code));
        };
    }
}
