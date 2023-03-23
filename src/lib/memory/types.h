#pragma once

#include <cstdint>
#include <span>
#include <vector>

using TByte = uint8_t;
using TWord = uint16_t;
using TLong = uint32_t;

using TAddressType = uint32_t;
using TAddressRange = std::pair<TAddressType, TAddressType>;

using TDataView = std::span<const TByte>;
using TDataHolder = std::vector<TByte>;
