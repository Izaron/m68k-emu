#pragma once

#include <lib/emulator/emulator.h>

namespace NOpcodes {

class TTarget {
public:
    enum EKind : uint8_t {
        DataRegisterKind,
        DecAddressRegisterKind,
    };

    void SetDataRegister(uint8_t index);
    void SetDecAddressRegisterKind(uint8_t index);

    // read data methods
    TDataHolder Read(NEmulator::TContext ctx, TAddressType size);
    TByte ReadByte(NEmulator::TContext ctx);
    TWord ReadWord(NEmulator::TContext ctx);
    TLong ReadLong(NEmulator::TContext ctx);

    // write data methods
    void Write(NEmulator::TContext ctx, TDataView data);
    void WriteByte(NEmulator::TContext ctx, TByte b);
    void WriteWord(NEmulator::TContext ctx, TWord w);
    void WriteLong(NEmulator::TContext ctx, TLong l);

private:
    union {
        uint8_t DataRegisterIndex;
        uint8_t DecAddressRegisterIndex;
    } Value_;
    EKind Kind_;
};

} // namespace NOpcodes
