#pragma once

#include <lib/emulator/emulator.h>

namespace NOpcodes {

class TTarget {
public:
    enum EKind : uint8_t {
        DataRegisterKind,
        AddressRegisterKind,
        AddressKind,
        AddressIncrementKind,
        AddressDecrementKind,
        AddressDisplacementKind,
        AddressIndexKind,
        ProgramCounterDisplacementKind,
        ProgramCounterIndexKind,
        AbsoluteShortKind,
        AbsoluteLongKind,
        ImmediateKind,
    };

    // builder methods
    void SetDataRegister(uint8_t index);
    void SetAddressRegister(uint8_t index);
    void SetAddress(uint8_t index);
    void SetAddressIncrement(uint8_t index);
    void SetAddressDecrement(uint8_t index);
    void SetAddressDisplacement(uint8_t index, TWord extWord0);
    void SetAddressIndex(uint8_t index, TWord extWord0);
    void SetProgramCounterDisplacement(TWord extWord0);
    void SetProgramCounterIndex(TWord extWord0);
    void SetAbsoluteShort(TWord extWord0);
    void SetAbsoluteLong(TWord extWord0, TWord extWord1);
    void SetImmediate(TLong address);

    // pre-work and post-work
    void TryDecrementAddress(NEmulator::TContext ctx);
    void TryIncrementAddress(NEmulator::TContext ctx);

    // read methods
    TDataHolder Read(NEmulator::TContext ctx, TAddressType size);
    TByte ReadByte(NEmulator::TContext ctx);
    TWord ReadWord(NEmulator::TContext ctx);
    TLong ReadLong(NEmulator::TContext ctx);

    // write methods
    void Write(NEmulator::TContext ctx, TDataView data);
    void WriteByte(NEmulator::TContext ctx, TByte b);
    void WriteWord(NEmulator::TContext ctx, TWord w);
    void WriteLong(NEmulator::TContext ctx, TLong l);

private:
    TLong GetIndexedAddress(NEmulator::TContext ctx, TLong baseAddress);

private:
    EKind Kind_;
    uint8_t Index_;
    TWord ExtWord0_;
    TWord ExtWord1_;
    TLong Address_;
};

} // namespace NOpcodes
