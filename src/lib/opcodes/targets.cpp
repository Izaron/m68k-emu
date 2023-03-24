#include "targets.h"

#include <type_traits>
#include <stdexcept>

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

void TTarget::SetDataRegister(uint8_t index) {
    Kind_ = DataRegisterKind;
    Index_ = index;
}

void TTarget::SetAddressRegister(uint8_t index) {
    Kind_ = AddressRegisterKind;
    Index_ = index;
}

void TTarget::SetAddress(uint8_t index) {
    Kind_ = AddressKind;
    Index_ = index;
}

void TTarget::SetAddressIncrement(uint8_t index) {
    Kind_ = AddressIncrementKind;
    Index_ = index;
}

void TTarget::SetAddressDecrement(uint8_t index) {
    Kind_ = AddressDecrementKind;
    Index_ = index;
}

void TTarget::SetAddressDisplacement(uint8_t index, TWord extWord0) {
    Kind_ = AddressDisplacementKind;
    Index_ = index;
    ExtWord0_ = extWord0;
}

void TTarget::SetAddressIndex(uint8_t index, TWord extWord0) {
    Kind_ = AddressIndexKind;
    Index_ = index;
    ExtWord0_ = extWord0;
}

void TTarget::SetProgramCounterDisplacement(TWord extWord0) {
    Kind_ = ProgramCounterDisplacementKind;
    ExtWord0_ = extWord0;
}

void TTarget::SetProgramCounterIndex(TWord extWord0) {
    Kind_ = ProgramCounterIndexKind;
    ExtWord0_ = extWord0;
}

void TTarget::SetAbsoluteShort(TWord extWord0) {
    Kind_ = AbsoluteShortKind;
    ExtWord0_ = extWord0;
}

void TTarget::SetAbsoluteLong(TWord extWord0, TWord extWord1) {
    Kind_ = AbsoluteLongKind;
    ExtWord0_ = extWord0;
    ExtWord1_ = extWord1;
}

void TTarget::SetImmediate(TLong address) {
    Kind_ = ImmediateKind;
    Address_ = address;
}

void TTarget::TryDecrementAddress(NEmulator::TContext ctx) {
    if (Kind_ == AddressDecrementKind) {
        auto& reg = GetAReg(ctx.Registers, Index_);
        --reg;

        // If the address register is the
        // stack pointer and the operand size is byte, the address is decremented by two to keep the
        // stack pointer aligned to a word boundary
        if (Index_ == 7) {
            --reg;
        }
    }
}

void TTarget::TryIncrementAddress(NEmulator::TContext ctx) {
    if (Kind_ == AddressIncrementKind) {
        auto& reg = GetAReg(ctx.Registers, Index_);
        ++reg;

        // If the address register is the
        // stack pointer and the operand size is byte, the address is decremented by two to keep the
        // stack pointer aligned to a word boundary
        if (Index_ == 7) {
            ++reg;
        }
    }
}

TDataHolder TTarget::Read(NEmulator::TContext ctx, TAddressType size) {
    TryDecrementAddress(ctx);

    const auto readRegister = [size](TLong reg) {
        TDataHolder data;
        for (int i = 0; i < size; ++i) {
            data.push_back(reg & 0xFF);
            reg >>= 8;
        }
        // TODO: reverse `data`?
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
        case AddressKind:
        case AddressIncrementKind:
        case AddressDecrementKind: {
            const auto reg = GetAReg(ctx.Registers, Index_);
            data = ctx.Memory.Read(reg, size);
            break;
        }
        case AddressDisplacementKind: {
            const auto reg = GetAReg(ctx.Registers, Index_);
            data = ctx.Memory.Read(reg + static_cast<TSignedWord>(ExtWord0_), size);
            break;
        }
        case AddressIndexKind: {
            const TByte displacement = GetBits(ExtWord0_, 0, 8);
            const uint8_t index = GetBits(ExtWord0_, 12, 3);
            const TLong indexReg = GetBit(ExtWord0_, 15) ? GetAReg(ctx.Registers, index) : ctx.Registers.D[index];
            const TLong scale = GetScaleValue(GetBits(ExtWord0_, 9, 2));
            const TLong reg = GetAReg(ctx.Registers, Index_);
            const TLong addr = reg + static_cast<TSignedByte>(displacement) + static_cast<TSignedLong>(indexReg) * scale;
            data = ctx.Memory.Read(addr, size);
            break;
        }
        case ProgramCounterDisplacementKind: {
            data = ctx.Memory.Read(ctx.Registers.PC - 2 + static_cast<TSignedWord>(ExtWord0_), size);
            break;
        }
        case ProgramCounterIndexKind: {
            const TByte displacement = GetBits(ExtWord0_, 0, 8);
            const uint8_t index = GetBits(ExtWord0_, 12, 3);
            const TLong indexReg = GetBit(ExtWord0_, 15) ? GetAReg(ctx.Registers, index) : ctx.Registers.D[index];
            const TLong scale = GetScaleValue(GetBits(ExtWord0_, 9, 2));
            const TLong addr = ctx.Registers.PC - 2 + static_cast<TSignedByte>(displacement) + indexReg * scale;
            data = ctx.Memory.Read(addr, size);
            break;
        }
        case AbsoluteShortKind: {
            data = ctx.Memory.Read(static_cast<TSignedWord>(ExtWord0_), size);
            break;
        }
        case AbsoluteLongKind: {
            data = ctx.Memory.Read((ExtWord0_ << 16) + ExtWord1_, size);
            break;
        }
        case ImmediateKind: {
            data = ctx.Memory.Read(Address_, size);
            break;
        }
        default: {
            throw std::runtime_error("target not supported");
        }
    }

    return data;
}

TByte TTarget::ReadByte(NEmulator::TContext ctx) {
    return Read(ctx, /*size=*/1)[0];
}

TWord TTarget::ReadWord(NEmulator::TContext ctx) {
    const auto data = Read(ctx, /*size=*/2);
    TWord res = data[0];
    res = (res << 8) + data[1];
    return res;
}

TLong TTarget::ReadLong(NEmulator::TContext ctx) {
    const auto data = Read(ctx, /*size=*/4);
    TLong res = data[0];
    res = (res << 8) + data[1];
    res = (res << 8) + data[2];
    res = (res << 8) + data[3];
    return res;
}

void TTarget::Write(NEmulator::TContext ctx, TDataView data) {
    const auto writeRegister = [data](TLong& reg) {
        int shift = 0;
        int lsb = 0;
        for (const auto value : data) {
            shift += 8;
            lsb <<= 8;
            lsb += value;
        }

        reg >>= shift;
        reg <<= shift;
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
            ctx.Memory.Write(reg, data);
            break;
        }
        case AddressDisplacementKind: {
            const auto reg = GetAReg(ctx.Registers, Index_);
            ctx.Memory.Write(reg + static_cast<TSignedWord>(ExtWord0_), data);
            break;
        }
        case AddressIndexKind: {
            const TByte displacement = GetBits(ExtWord0_, 0, 8);
            const uint8_t index = GetBits(ExtWord0_, 12, 3);
            const TLong indexReg = GetBit(ExtWord0_, 15) ? GetAReg(ctx.Registers, index) : ctx.Registers.D[index];
            const TLong scale = GetScaleValue(GetBits(ExtWord0_, 9, 2));
            const TLong reg = GetAReg(ctx.Registers, Index_);
            const TLong addr = reg + static_cast<TSignedByte>(displacement) + static_cast<TSignedLong>(indexReg) * scale;
            ctx.Memory.Write(addr, data);
            break;
        }
        case ProgramCounterDisplacementKind: {
            ctx.Memory.Write(ctx.Registers.PC - 2 + static_cast<TSignedWord>(ExtWord0_), data);
            break;
        }
        case ProgramCounterIndexKind: {
            const TByte displacement = GetBits(ExtWord0_, 0, 8);
            const uint8_t index = GetBits(ExtWord0_, 12, 3);
            const TLong indexReg = GetBit(ExtWord0_, 15) ? GetAReg(ctx.Registers, index) : ctx.Registers.D[index];
            const TLong scale = GetScaleValue(GetBits(ExtWord0_, 9, 2));
            const TLong addr = ctx.Registers.PC - 2 + static_cast<TSignedByte>(displacement) + indexReg * scale;
            ctx.Memory.Write(addr, data);
            break;
        }
        case AbsoluteShortKind: {
            ctx.Memory.Write(static_cast<TSignedWord>(ExtWord0_), data);
            break;
        }
        case AbsoluteLongKind: {
            ctx.Memory.Write((ExtWord0_ << 16) + ExtWord1_, data);
            break;
        }
        case ImmediateKind: {
            ctx.Memory.Write(Address_, data);
            break;
        }
        default: {
            break;
        }
    }
}

void TTarget::WriteByte(NEmulator::TContext ctx, TByte b) {
    TDataHolder data;
    data.emplace_back(b);
    return Write(ctx, data);
}

void TTarget::WriteWord(NEmulator::TContext ctx, TWord w) {
    TDataHolder data;
    for (int i = 0; i < 2; ++i) {
        data.emplace_back(w & 0xFF);
        w >>= 8;
    }
    return Write(ctx, data);
}

void TTarget::WriteLong(NEmulator::TContext ctx, TLong l) {
    TDataHolder data;
    for (int i = 0; i < 4; ++i) {
        data.emplace_back(l & 0xFF);
        l >>= 8;
    }
    return Write(ctx, data);
}

} // namespace NOpcodes
