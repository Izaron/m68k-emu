#pragma once

#include <optional>

#include <lib/error/error.h>
#include <thirdparty/include/tl/expected.hpp>

#include "types.h"

namespace NMemory {

class IDevice {
public:
    virtual tl::expected<TDataHolder, TError> Read(TAddressType addr, TAddressType size) = 0;
    [[nodiscard]] virtual std::optional<TError> Write(TAddressType addr, TDataView data) = 0;

    // helpers
    tl::expected<TByte, TError> ReadByte(TAddressType addr);
    tl::expected<TWord, TError> ReadWord(TAddressType addr);
    tl::expected<TLong, TError> ReadLong(TAddressType addr);

    [[nodiscard]] std::optional<TError> WriteByte(TAddressType addr, TByte b);
    [[nodiscard]] std::optional<TError> WriteWord(TAddressType addr, TWord w);
    [[nodiscard]] std::optional<TError> WriteLong(TAddressType addr, TLong l);
};

} // namespace NMemory
