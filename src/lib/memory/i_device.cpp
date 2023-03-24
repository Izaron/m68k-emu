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

} // namespace NMemory
