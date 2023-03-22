#pragma once

#include <cstdint>
#include <span>
#include <vector>

namespace NMemory {

using TAddressType = uint32_t;
using TAddressRange = std::pair<TAddressType, TAddressType>;

using TDataType = std::byte;
using TDataView = std::span<const TDataType>;
using TDataHolder = std::vector<TDataType>;

} // namespace NMemory
