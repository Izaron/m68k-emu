#include "targets.h"

#include <algorithm>
#include <type_traits>

namespace NOpcodes {

static_assert(std::is_trivially_constructible_v<TTarget>);

namespace {

TLong& GetAReg(NRegisters::TRegisters& r, int index) {
    if (index < 7) {
        return r.A[index];
    } else {
        return r.GetSupervisorFlag() ? r.SSP : r.USP;
    }
}

int8_t GetBits(auto value, std::size_t begin, std::size_t len) {
    return (value >> begin) & ((1 << len) - 1);
}

int8_t GetBit(auto value, std::size_t bit) {
    return GetBits(value, bit, 1);
}

int8_t GetScaleValue(int8_t mode) {
    return 1;

    switch (mode) {
        case 0: return 1;
        case 1: return 2;
        case 2: return 4;
        case 3: return 8;
        default: __builtin_unreachable();
    }
}

} // namespace

TTarget& TTarget::SetKind(EKind kind) {
    Kind_ = kind;
    if (Kind_ == AddressDecrementKind) {
        AlreadyDecremented_ = false;
    }
    return *this;
}

TTarget& TTarget::SetSize(uint8_t size) {
    Size_ = size;
    return *this;
}

TTarget& TTarget::SetIndex(uint8_t index) {
    Index_ = index;
    return *this;
}

TTarget& TTarget::SetExtWord0(TWord extWord0) {
    ExtWord0_ = extWord0;
    return *this;
}

TTarget& TTarget::SetExtWord1(TWord extWord1) {
    ExtWord1_ = extWord1;
    return *this;
}

TTarget& TTarget::SetAddress(TLong address) {
    Address_ = address;
    return *this;
}

void TTarget::SetIncOrDecCount(std::size_t count) {
    IncOrDecCount_ = count;
}

void TTarget::TryDecrementAddress(NEmulator::TContext ctx, std::size_t count) {
    if (Kind_ == AddressDecrementKind && !AlreadyDecremented_) {
        auto& reg = GetAReg(ctx.Registers, Index_);

        // stack pointer should be aligned to a word boundary
        TLong diff = Size_ * count;
        reg -= (Index_ == 7) ? std::max((int)diff, 2) : diff;
    }
    AlreadyDecremented_ = true;
}

void TTarget::TryIncrementAddress(NEmulator::TContext ctx, std::size_t count) {
    if (Kind_ == AddressIncrementKind) {
        auto& reg = GetAReg(ctx.Registers, Index_);

        // stack pointer should be aligned to a word boundary
        TLong diff = Size_ * count;
        reg += (Index_ == 7) ? std::max((int)diff, 2) : diff;
    }
}

TLong TTarget::GetEffectiveAddress(NEmulator::TContext ctx) const {
    switch (Kind_) {
        case AddressKind:
        case AddressIncrementKind:
        case AddressDecrementKind:
            return GetAReg(ctx.Registers, Index_);
        case AddressDisplacementKind:
            return GetAReg(ctx.Registers, Index_) + static_cast<TSignedWord>(ExtWord0_);
        case AddressIndexKind:
            return GetIndexedAddress(ctx, GetAReg(ctx.Registers, Index_));
        case ProgramCounterDisplacementKind:
            return ctx.Registers.PC - 2 + static_cast<TSignedWord>(ExtWord0_);
        case ProgramCounterIndexKind:
            return GetIndexedAddress(ctx, ctx.Registers.PC - 2);
        case AbsoluteShortKind:
            return static_cast<TSignedWord>(ExtWord0_);
        case AbsoluteLongKind:
            return (ExtWord0_ << 16) + ExtWord1_;
        case ImmediateKind:
            return Address_;
        default:
            __builtin_unreachable();
    }
}

tl::expected<TDataHolder, TError> TTarget::Read(NEmulator::TContext ctx, TAddressType size) {
    TryDecrementAddress(ctx, IncOrDecCount_);

    const auto readRegister = [size](TLong reg) {
        TDataHolder data;
        for (int i = 0; i < size; ++i) {
            data.push_back(reg & 0xFF);
            reg >>= 8;
        }
        std::reverse(data.begin(), data.end());
        return data;
    };

    TDataHolder data;

    switch (Kind_) {
        case DataRegisterKind: {
            data = readRegister(ctx.Registers.D[Index_]);
            break;
        }
        case AddressRegisterKind: {
            data = readRegister(GetAReg(ctx.Registers, Index_));
            break;
        }
        case AbsoluteLongKind:
        case AbsoluteShortKind:
        case AddressDecrementKind:
        case AddressDisplacementKind:
        case AddressIncrementKind:
        case AddressIndexKind:
        case AddressKind:
        case ImmediateKind:
        case ProgramCounterDisplacementKind:
        case ProgramCounterIndexKind: {
            auto dataOrError = ctx.Memory.Read(GetEffectiveAddress(ctx), size);
            if (!dataOrError) { return tl::unexpected(dataOrError.error()); }
            data = std::move(*dataOrError);
            break;
        }
    }

    return data;
}

tl::expected<TLongLong, TError> TTarget::ReadAsLongLong(NEmulator::TContext ctx, TAddressType size) {
    return Read(ctx, size).map([size](auto&& data) {
        TLongLong res = data[0];
        for (int i = 1; i < size; ++i) {
            res = (res << 8) + data[i];
        }
        return res;
    });
}

tl::expected<TByte, TError> TTarget::ReadByte(NEmulator::TContext ctx) {
    return Read(ctx, /*size=*/1).map([](auto&& data) { return data[0]; });
}

tl::expected<TWord, TError> TTarget::ReadWord(NEmulator::TContext ctx) {
    return Read(ctx, /*size=*/2).map([](auto&& data) {
        TWord res = data[0];
        res = (res << 8) + data[1];
        return res;
    });
}

tl::expected<TLong, TError> TTarget::ReadLong(NEmulator::TContext ctx) {
    return Read(ctx, /*size=*/4).map([](auto&& data) {
        TLong res = data[0];
        res = (res << 8) + data[1];
        res = (res << 8) + data[2];
        res = (res << 8) + data[3];
        return res;
    });
}

std::optional<TError> TTarget::Write(NEmulator::TContext ctx, TDataView data) {
    TryDecrementAddress(ctx, IncOrDecCount_);

    const auto writeRegister = [data](TLong& reg) {
        TLong shift = 0;
        TLong lsb = 0;
        for (const auto value : data) {
            shift += 8;
            lsb <<= 8;
            lsb += value;
        }

        if (shift == 32) {
            reg = 0;
        } else {
            reg >>= shift;
            reg <<= shift;
        }
        reg |= lsb;
    };

    switch (Kind_) {
        case DataRegisterKind: {
            writeRegister(ctx.Registers.D[Index_]);
            break;
        }
        case AddressRegisterKind: {
            writeRegister(GetAReg(ctx.Registers, Index_));
            break;
        }
        case AddressKind:
        case AddressIncrementKind:
        case AddressDecrementKind: {
            const auto reg = GetAReg(ctx.Registers, Index_);
            return ctx.Memory.Write(reg, data);
        }
        case AddressDisplacementKind: {
            const auto reg = GetAReg(ctx.Registers, Index_);
            return ctx.Memory.Write(reg + static_cast<TSignedWord>(ExtWord0_), data);
        }
        case AddressIndexKind: {
            const TLong addr = GetIndexedAddress(ctx, GetAReg(ctx.Registers, Index_));
            return ctx.Memory.Write(addr, data);
        }
        case ProgramCounterDisplacementKind: {
            return ctx.Memory.Write(ctx.Registers.PC - 2 + static_cast<TSignedWord>(ExtWord0_), data);
        }
        case ProgramCounterIndexKind: {
            const TLong addr = GetIndexedAddress(ctx, ctx.Registers.PC - 2);
            return ctx.Memory.Write(addr, data);
        }
        case AbsoluteShortKind: {
            return ctx.Memory.Write(static_cast<TSignedWord>(ExtWord0_), data);
        }
        case AbsoluteLongKind: {
            return ctx.Memory.Write((ExtWord0_ << 16) + ExtWord1_, data);
        }
        case ImmediateKind: {
            return ctx.Memory.Write(Address_, data);
        }
    }

    return std::nullopt;
}

std::optional<TError> TTarget::WriteSized(NEmulator::TContext ctx, TLong value, TAddressType size) {
    switch (size) {
        case 1: return WriteByte(ctx, value);
        case 2: return WriteWord(ctx, value);
        case 4: return WriteLong(ctx, value);
        default: __builtin_unreachable();
    }
}

std::optional<TError> TTarget::WriteByte(NEmulator::TContext ctx, TByte b) {
    TDataHolder data;
    data.emplace_back(b);
    return Write(ctx, data);
}

std::optional<TError> TTarget::WriteWord(NEmulator::TContext ctx, TWord w) {
    TDataHolder data;
    for (int i = 0; i < 2; ++i) {
        data.emplace_back(w & 0xFF);
        w >>= 8;
    }
    std::reverse(data.begin(), data.end());
    return Write(ctx, data);
}

std::optional<TError> TTarget::WriteLong(NEmulator::TContext ctx, TLong l) {
    TDataHolder data;
    for (int i = 0; i < 4; ++i) {
        data.emplace_back(l & 0xFF);
        l >>= 8;
    }
    std::reverse(data.begin(), data.end());
    return Write(ctx, data);
}

TLong TTarget::GetIndexedAddress(NEmulator::TContext ctx, TLong baseAddress) const {
    const uint8_t xregNum = GetBits(ExtWord0_, 12, 3);
    const TLong xreg = GetBit(ExtWord0_, 15) ? GetAReg(ctx.Registers, xregNum) : ctx.Registers.D[xregNum];
    const TLong size = GetBit(ExtWord0_, 11) ? /*Long*/ 4 : /*Word*/ 2;
    const TLong scale = GetScaleValue(GetBits(ExtWord0_, 9, 2));
    const TSignedByte disp = static_cast<TSignedByte>(GetBits(ExtWord0_, 0, 8));

    TSignedLong clarifiedXreg = xreg;
    if (size == 2) {
        clarifiedXreg = static_cast<TSignedWord>(clarifiedXreg);
    }

    return baseAddress + disp + clarifiedXreg * scale;
}

} // namespace NOpcodes
