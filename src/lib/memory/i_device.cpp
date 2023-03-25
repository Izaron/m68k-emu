#include "i_device.h"

namespace NMemory {

tl::expected<TByte, TError> IDevice::ReadByte(TAddressType addr) {
    return Read(addr, /*size=*/1).map([](auto&& data) { return data[0]; });
}

tl::expected<TWord, TError> IDevice::ReadWord(TAddressType addr) {
    return Read(addr, /*size=*/2).map([](auto&& data) {
        TWord res = data[0];
        res = (res << 8) + data[1];
        return res;
    });
}

tl::expected<TLong, TError> IDevice::ReadLong(TAddressType addr) {
    return Read(addr, /*size=*/4).map([](auto&& data) {
        TLong res = data[0];
        res = (res << 8) + data[1];
        res = (res << 8) + data[2];
        res = (res << 8) + data[3];
        return res;
    });
}

std::optional<TError> IDevice::WriteByte(TAddressType addr, TByte b) {
    TDataHolder data;
    data.emplace_back(b);
    return Write(addr, data);
}

std::optional<TError> IDevice::WriteWord(TAddressType addr, TWord w) {
    TDataHolder data;
    for (int i = 0; i < 2; ++i) {
        data.emplace_back(w & 0xFF);
        w >>= 8;
    }
    std::reverse(data.begin(), data.end());
    return Write(addr, data);
}

std::optional<TError> IDevice::WriteLong(TAddressType addr, TLong l) {
    TDataHolder data;
    for (int i = 0; i < 4; ++i) {
        data.emplace_back(l & 0xFF);
        l >>= 8;
    }
    std::reverse(data.begin(), data.end());
    return Write(addr, data);
}

} // namespace NMemory
