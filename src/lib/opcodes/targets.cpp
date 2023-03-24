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

void TTarget::TryDecrementAddress(NEmulator::TContext ctx) {
    if (Kind_ == AddressDecrementKind) {
        auto& reg = GetAReg(ctx.Registers, Index_);

        // stack pointer should be aligned to a word boundary
        reg -= (Index_ == 7) ? std::max((int)Size_, 2) : Size_;
    }
}

void TTarget::TryIncrementAddress(NEmulator::TContext ctx) {
    if (Kind_ == AddressIncrementKind) {
        auto& reg = GetAReg(ctx.Registers, Index_);

        // stack pointer should be aligned to a word boundary
        reg += (Index_ == 7) ? std::max((int)Size_, 2) : Size_;
    }
}

tl::expected<TDataHolder, TError> TTarget::Read(NEmulator::TContext ctx, TAddressType size) {
    TryDecrementAddress(ctx);

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

#define GET_DATA_SAFE \
    if (!dataOrError) { return tl::unexpected(dataOrError.error()); } \
    data = std::move(*dataOrError)

    switch (Kind_) {
        case DataRegisterKind: {
            data = readRegister(ctx.Registers.D[Index_]);
            break;
        }
        case AddressRegisterKind: {
            data = readRegister(GetAReg(ctx.Registers, Index_));
            break;
        }
        case AddressKind:
        case AddressIncrementKind:
        case AddressDecrementKind: {
            const auto reg = GetAReg(ctx.Registers, Index_);
            auto dataOrError = ctx.Memory.Read(reg, size);
            GET_DATA_SAFE;
            break;
        }
        case AddressDisplacementKind: {
            const auto reg = GetAReg(ctx.Registers, Index_);
            auto dataOrError = ctx.Memory.Read(reg + static_cast<TSignedWord>(ExtWord0_), size);
            GET_DATA_SAFE;
            break;
        }
        case AddressIndexKind: {
            const TLong addr = GetIndexedAddress(ctx, GetAReg(ctx.Registers, Index_));
            auto dataOrError = ctx.Memory.Read(addr, size);
            GET_DATA_SAFE;
            break;
        }
        case ProgramCounterDisplacementKind: {
            auto dataOrError = ctx.Memory.Read(ctx.Registers.PC - 2 + static_cast<TSignedWord>(ExtWord0_), size);
            GET_DATA_SAFE;
            break;
        }
        case ProgramCounterIndexKind: {
            const TLong addr = GetIndexedAddress(ctx, ctx.Registers.PC - 2);
            auto dataOrError = ctx.Memory.Read(addr, size);
            GET_DATA_SAFE;
            break;
        }
        case AbsoluteShortKind: {
            auto dataOrError = ctx.Memory.Read(static_cast<TSignedWord>(ExtWord0_), size);
            GET_DATA_SAFE;
            break;
        }
        case AbsoluteLongKind: {
            auto dataOrError = ctx.Memory.Read((ExtWord0_ << 16) + ExtWord1_, size);
            GET_DATA_SAFE;
            break;
        }
        case ImmediateKind: {
            auto dataOrError = ctx.Memory.Read(Address_, size);
            GET_DATA_SAFE;
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

TLong TTarget::GetIndexedAddress(NEmulator::TContext ctx, TLong baseAddress) {
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
