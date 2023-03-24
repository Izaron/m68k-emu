#include "instructions.h"

#include <type_traits>
#include <iostream>

namespace NOpcodes {

static_assert(std::is_trivially_constructible_v<TInstruction>);

namespace {

template<typename Type, typename Value>
bool GetMostSignificantBit(Value value) {
    if constexpr (std::is_same_v<Type, TByte>) {
        return value & (1 << 7);
    }
    else if constexpr (std::is_same_v<Type, TWord>) {
        return value & (1 << 15);
    }
    else if constexpr (std::is_same_v<Type, TLong>) {
        return value & (1 << 31);
    }
}

bool IsCarry(TLongLong value, TInstruction::ESize size) {
    switch (size) {
        case TInstruction::Byte: return value & (value ^ 0xFF);
        case TInstruction::Word: return value & (value ^ 0xFFFF);
        case TInstruction::Long: return value & (value ^ 0xFFFFFFFF);
        default: __builtin_unreachable();
    }
}

bool IsZero(TLongLong value, TInstruction::ESize size) {
    switch (size) {
        case TInstruction::Byte: return (value & 0xFF) == 0;
        case TInstruction::Word: return (value & 0xFFFF) == 0;
        case TInstruction::Long: return (value & 0xFFFFFFFF) == 0;
        default: __builtin_unreachable();
    }
}

bool GetMsb(TLongLong value, TInstruction::ESize size) {
    switch (size) {
        case TInstruction::Byte: return (value >> 7) & 1;
        case TInstruction::Word: return (value >> 15) & 1;
        case TInstruction::Long: return (value >> 31) & 1;
        default: __builtin_unreachable();
    }
}

bool IsOverflow(TLongLong lhs, TLongLong rhs, TLongLong result, TInstruction::ESize size) {
    const bool lhsMsb = GetMsb(lhs, size);
    const bool rhsMsb = GetMsb(rhs, size);
    const bool resultMsb = GetMsb(result, size);
    return (lhsMsb && rhsMsb && !resultMsb) || (!lhsMsb && !rhsMsb && resultMsb);
}

} // namespace

void TInstruction::SetAbcd(TTarget src, TTarget dst) {
    Kind_ = AbcdKind;
    Src_ = src;
    Dst_ = dst;
    HasSrc_ = HasDst_ = true;
}

void TInstruction::SetAdd(TTarget src, TTarget dst, ESize size) {
    Kind_ = AddKind;
    Src_ = src;
    Dst_ = dst;
    Size_ = size;
    HasSrc_ = HasDst_ = true;
}

void TInstruction::SetAddi(TTarget src, TTarget dst, ESize size) {
    Kind_ = AddiKind;
    Src_ = src;
    Dst_ = dst;
    Size_ = size;
    HasSrc_ = HasDst_ = true;
}

void TInstruction::SetAddq(TWord data, TTarget dst, ESize size) {
    Kind_ = AddqKind;
    Data_ = data;
    Dst_ = dst;
    Size_ = size;
    HasSrc_ = false;
    HasDst_ = true;
}

void TInstruction::SetNop() {
    Kind_ = NopKind;
    HasSrc_ = HasDst_ = false;
}

std::optional<TError> TInstruction::Execute(NEmulator::TContext ctx) {

#define SAFE_CALL(arg)                              \
    if (auto err = arg) { return std::move(arg); }

#define SAFE_DECLARE(name, init)                    \
    const auto name = init;                         \
    if (!name) { return name.error(); }

    switch (Kind_) {
        case NopKind: {
            break;
        }
        case AbcdKind: {
            SAFE_DECLARE(srcVal, Src_.ReadByte(ctx));
            SAFE_DECLARE(dstVal, Dst_.ReadByte(ctx));
            const TByte extendFlag = ctx.Registers.GetExtendFlag();

            const TWord binaryResult = *srcVal + *dstVal + extendFlag;

            bool carry = false;
            int lval = (*srcVal & 0x0F) + (*dstVal & 0x0F) + extendFlag;
            if (lval > 9) {
                carry = true;
                lval -= 10;
            }

            int hval = ((*srcVal >> 4) & 0x0F) + ((*dstVal >> 4) & 0x0F) + (carry ? 1 : 0);
            carry = false;

            if (lval >= 16) {
                lval -= 16;
                ++hval;
            }

            if (hval > 9) {
                carry = true;
                hval -= 10;
            }

            const TWord result = ((hval << 4) + lval) & 0xFF;

            SAFE_CALL(Dst_.WriteByte(ctx, result));
            ctx.Registers.SetNegativeFlag(GetMostSignificantBit<TByte>(result));
            ctx.Registers.SetCarryFlag(carry);
            ctx.Registers.SetExtendFlag(carry);
            ctx.Registers.SetOverflowFlag((~binaryResult & result & 0x80) != 0);
            if (result != 0) {
                ctx.Registers.SetZeroFlag(result == 0);
            }
            break;
        }
        case AddiKind:
        case AddKind: {
            SAFE_DECLARE(srcVal, Src_.ReadAsLongLong(ctx, Size_));
            SAFE_DECLARE(dstVal, Dst_.ReadAsLongLong(ctx, Size_));
            std::cerr << "SrcVal " << *srcVal << std::endl;
            std::cerr << "DstVal " << *dstVal << std::endl;
            const TLongLong result = *srcVal + *dstVal;
            std::cerr << "Result " << result << std::endl;
            SAFE_CALL(Dst_.WriteSized(ctx, result, Size_));

            const bool carry = IsCarry(result, Size_);
            ctx.Registers.SetNegativeFlag(GetMsb(result, Size_));
            ctx.Registers.SetCarryFlag(carry);
            ctx.Registers.SetExtendFlag(carry);
            ctx.Registers.SetOverflowFlag(IsOverflow(*srcVal, *dstVal, result, Size_));
            ctx.Registers.SetZeroFlag(IsZero(result, Size_));
            break;
        }
        case AddqKind: {
            const TLongLong srcVal = Data_ ? Data_ : 8;
            SAFE_DECLARE(dstVal, Dst_.ReadAsLongLong(ctx, Size_));
            const TLongLong result = srcVal + *dstVal;
            SAFE_CALL(Dst_.WriteSized(ctx, result, Size_));

            if (Dst_.GetKind() != TTarget::AddressRegisterKind) {
                const bool carry = IsCarry(result, Size_);
                ctx.Registers.SetNegativeFlag(GetMsb(result, Size_));
                ctx.Registers.SetCarryFlag(carry);
                ctx.Registers.SetExtendFlag(carry);
                ctx.Registers.SetOverflowFlag(IsOverflow(srcVal, *dstVal, result, Size_));
                ctx.Registers.SetZeroFlag(IsZero(result, Size_));
            }
            break;
        }
    }

    if (HasSrc_) {
        Src_.TryIncrementAddress(ctx);
    }
    if (HasDst_) {
        Dst_.TryIncrementAddress(ctx);
    }

    return std::nullopt;
}

tl::expected<TInstruction, TError> TInstruction::Decode(NEmulator::TContext ctx) {
    const auto readWord = [&ctx]() {
        auto& pc = ctx.Registers.PC;
        const auto word = ctx.Memory.ReadWord(pc);
        if (word) {
            pc += 2;
        }
        return word;
    };

#define READ_WORD_SAFE                       \
    const auto word = readWord();            \
    if (!word) {                             \
        return tl::unexpected(word.error()); \
    }

#define READ_TWO_WORDS_SAFE                                 \
    const auto word0 = readWord();                          \
    if (!word0) { return tl::unexpected(word0.error()); }   \
    const auto word1 = readWord();                          \
    if (!word1) { return tl::unexpected(word0.error()); }

    // read the current word (16 bits)
    READ_WORD_SAFE;

    // helper functions
    const auto applyMask = [&word](TWord mask) { return *word & mask; };
    const auto getBits = [&word](std::size_t begin, std::size_t len) { return (*word >> begin) & ((1 << len) - 1); };
    const auto getBit = [&getBits](std::size_t bit) { return getBits(bit, 1); };

    const auto getSize0 = [&getBits]() {
        // 00 -> byte, 01 -> word, 02 -> long
        switch (getBits(6, 2)) {
            case 0: return Byte;
            case 1: return Word;
            case 2: return Long;
            default: __builtin_unreachable();
        }
    };

    const auto parseDst = [&]() -> tl::expected<TTarget, TError> {
        TTarget dst;

        const auto mode = getBits(3, 3);
        const auto xn = getBits(0, 3);

        switch (mode) {
            case 0: {
                dst.SetKind(TTarget::DataRegisterKind).SetIndex(xn);
                break;
            }
            case 1: {
                dst.SetKind(TTarget::AddressRegisterKind).SetIndex(xn);
                break;
            }
            case 2: {
                dst.SetKind(TTarget::AddressKind).SetIndex(xn);
                break;
            }
            case 3: {
                dst.SetKind(TTarget::AddressIncrementKind).SetIndex(xn).SetSize(getSize0());
                break;
            }
            case 4: {
                dst.SetKind(TTarget::AddressDecrementKind).SetIndex(xn).SetSize(getSize0());
                break;
            }
            case 5: {
                READ_WORD_SAFE;
                dst.SetKind(TTarget::AddressDisplacementKind).SetIndex(xn).SetExtWord0(*word);
                break;
            }
            case 6: {
                READ_WORD_SAFE;
                dst.SetKind(TTarget::AddressIndexKind).SetIndex(xn).SetExtWord0(*word);
                break;
            }
            case 7: {
                switch (xn) {
                    case 0: {
                        READ_WORD_SAFE;
                        dst.SetKind(TTarget::AbsoluteShortKind).SetExtWord0(*word);
                        break;
                    }
                    case 1: {
                        READ_TWO_WORDS_SAFE;
                        dst.SetKind(TTarget::AbsoluteLongKind).SetExtWord0(*word0).SetExtWord1(*word1);
                        break;
                    }
                    case 2: {
                        READ_WORD_SAFE;
                        dst.SetKind(TTarget::ProgramCounterDisplacementKind).SetExtWord0(*word);
                        break;
                    }
                    case 3: {
                        READ_WORD_SAFE;
                        dst.SetKind(TTarget::ProgramCounterIndexKind).SetExtWord0(*word);
                        break;
                    }
                    case 4: {
                        auto& pc = ctx.Registers.PC;
                        dst.SetKind(TTarget::ImmediateKind).SetAddress((getSize0() == Byte) ? (pc + 1) : pc);
                        pc += (getSize0() == Long) ? 4 : 2;
                        break;
                    }
                    default: {
                        return tl::unexpected<TError>(TError::UnknownAddressingMode, "Unknown addresing mode in word %#04x", *word);
                    }
                }
                break;
            }
            default: {
                __builtin_unreachable();
            }
        }

        return dst;
    };

    // decode the opcode
    TInstruction inst;

    if (applyMask(0b1111'1111'0000'0000) == 0b0000'0110'0000'0000) {
        TTarget src;
        auto& pc = ctx.Registers.PC;
        src.SetKind(TTarget::ImmediateKind).SetAddress((getSize0() == Byte) ? (pc + 1) : pc);
        pc += (getSize0() == Long) ? 4 : 2;

        const auto dst = parseDst();
        if (!dst) { return tl::unexpected(dst.error()); }
        inst.SetAddi(src, *dst, getSize0());
    }
    else if (*word == 0b0100'1100'0111'0001) {
        inst.SetNop();
    }
    else if (applyMask(0b1111'0001'0000'0000) == 0b0101'0000'0000'0000) {
        const auto dst = parseDst();
        if (!dst) { return tl::unexpected(dst.error()); }
        inst.SetAddq(getBits(9, 3), *dst, getSize0());
    }
    else if (applyMask(0b1111'0001'0000'0000) == 0b1100'0001'0000'0000) {
        const auto kind = getBit(3) ? TTarget::AddressDecrementKind : TTarget::DataRegisterKind;
        TTarget src;
        src.SetKind(kind).SetIndex(getBits(0, 3)).SetSize(1);
        TTarget dst;
        dst.SetKind(kind).SetIndex(getBits(9, 3)).SetSize(1);
        inst.SetAbcd(src, dst);
    }
    else if (applyMask(0b1111'0000'0000'0000) == 0b1101'0000'0000'0000) {
        TTarget src;
        src.SetKind(TTarget::DataRegisterKind).SetIndex(getBits(9, 3));

        auto dst = parseDst();
        if (!dst) { return tl::unexpected(dst.error()); }
        if (!getBit(8)) {
            std::swap(src, *dst);
        }
        inst.SetAdd(src, *dst, getSize0());
    }
    else {
        return tl::unexpected<TError>(TError::UnknownOpcode, "Unknown opcode %#04x", *word);
    }

    return inst;
}

} // namespace NOpcodes
