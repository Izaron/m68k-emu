#pragma once

#include <cstdint>
#include <span>
#include <vector>

using TByte = uint8_t;
using TWord = uint16_t;
using TLong = uint32_t;

using TSignedByte = int8_t;
using TSignedWord = int16_t;
using TSignedLong = int32_t;

using TAddressType = TLong;
using TAddressRange = std::pair<TAddressType, TAddressType>;

using TDataView = std::span<const TByte>;
using TDataHolder = std::vector<TByte>;
