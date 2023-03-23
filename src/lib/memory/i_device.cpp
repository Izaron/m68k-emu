#include "i_device.h"

namespace NMemory {

TByte IDevice::ReadByte(TAddressType addr) {
    return Read(addr, /*size=*/1)[0];
}

TWord IDevice::ReadWord(TAddressType addr) {
    const auto data = Read(addr, /*size=*/2);
    TWord res = data[0];
    res = (res << 8) + data[1];
    return res;
}

TLong IDevice::ReadLong(TAddressType addr) {
    const auto data = Read(addr, /*size=*/4);
    TLong res = data[0];
    res = (res << 8) + data[1];
    res = (res << 8) + data[2];
    res = (res << 8) + data[3];
    return res;
}

} // namespace NMemory
