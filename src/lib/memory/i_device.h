#pragma once

#include "types.h"

namespace NMemory {

class IDevice {
public:
    virtual TDataHolder Read(TAddressType addr, TAddressType size) = 0;
    virtual void Write(TAddressType addr, TDataView data) = 0;

    // helpers
    TByte ReadByte(TAddressType addr);
    TWord ReadWord(TAddressType addr);
    TLong ReadLong(TAddressType addr);
};

} // namespace NMemory
