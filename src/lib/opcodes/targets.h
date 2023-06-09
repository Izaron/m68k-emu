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
    TTarget& SetSize(uint8_t size);
    TTarget& SetIndex(uint8_t index);
    TTarget& SetExtWord0(TWord extWord0);
    TTarget& SetExtWord1(TWord extWord1);
    TTarget& SetAddress(TLong address);

    // pre-work and post-work
    void SetIncOrDecCount(std::size_t count);
    void TryDecrementAddress(NEmulator::TContext ctx, std::size_t count = 1);
    void TryIncrementAddress(NEmulator::TContext ctx, std::size_t count = 1);

    // helper methods
    TLong GetEffectiveAddress(NEmulator::TContext ctx) const;
    EKind GetKind() const { return Kind_; }
    uint8_t GetIndex() const { return Index_; }

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
    TLong GetIndexedAddress(NEmulator::TContext ctx, TLong baseAddress) const;

private:
    EKind Kind_;
    uint8_t Size_;
    uint8_t Index_;
    TWord ExtWord0_;
    TWord ExtWord1_;
    TLong Address_;

    bool AlreadyDecremented_;
    std::size_t IncOrDecCount_;
};

} // namespace NOpcodes
