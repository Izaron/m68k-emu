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

    TTarget& SetKind(EKind kind);
    TTarget& SetSize(uint8_t index);
    TTarget& SetIndex(uint8_t index);
    TTarget& SetExtWord0(TWord extWord0);
    TTarget& SetExtWord1(TWord extWord1);
    TTarget& SetAddress(TLong address);

    // pre-work and post-work
    void TryDecrementAddress(NEmulator::TContext ctx);
    void TryIncrementAddress(NEmulator::TContext ctx);

    // helper methods
    EKind GetKind() const { return Kind_; }

    // read methods
    tl::expected<TDataHolder, TError> Read(NEmulator::TContext ctx, TAddressType size);
    tl::expected<TLongLong, TError> ReadAsLongLong(NEmulator::TContext ctx, TAddressType size);
    tl::expected<TByte, TError> ReadByte(NEmulator::TContext ctx);
    tl::expected<TWord, TError> ReadWord(NEmulator::TContext ctx);
    tl::expected<TLong, TError> ReadLong(NEmulator::TContext ctx);

    // write methods
    [[nodiscard]] std::optional<TError> Write(NEmulator::TContext ctx, TDataView data);
    [[nodiscard]] std::optional<TError> WriteSized(NEmulator::TContext ctx, TLong value, TAddressType size);
    [[nodiscard]] std::optional<TError> WriteByte(NEmulator::TContext ctx, TByte b);
    [[nodiscard]] std::optional<TError> WriteWord(NEmulator::TContext ctx, TWord w);
    [[nodiscard]] std::optional<TError> WriteLong(NEmulator::TContext ctx, TLong l);

private:
    TLong GetIndexedAddress(NEmulator::TContext ctx, TLong baseAddress);

private:
    EKind Kind_;
    uint8_t Size_;
    uint8_t Index_;
    TWord ExtWord0_;
    TWord ExtWord1_;
    TLong Address_;
};

} // namespace NOpcodes
