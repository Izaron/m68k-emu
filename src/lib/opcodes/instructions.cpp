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

uint8_t BitCount(TInstruction::ESize size) {
    return size << 3;
}

bool GetMsb(TLongLong value, TInstruction::ESize size) {
    return (value >> (BitCount(size) - 1)) & 1;
}

bool IsOverflow(TLongLong lhs, TLongLong rhs, TLongLong result, TInstruction::ESize size) {
    const bool lhsMsb = GetMsb(lhs, size);
    const bool rhsMsb = GetMsb(rhs, size);
    const bool resultMsb = GetMsb(result, size);
    return (lhsMsb && rhsMsb && !resultMsb) || (!lhsMsb && !rhsMsb && resultMsb);
}

} // namespace

TInstruction& TInstruction::SetKind(EKind kind) {
    Kind_ = kind;
    HasSrc_ = HasDst_ = false;
    return *this;
}

TInstruction& TInstruction::SetSrc(TTarget target) {
    Src_ = target;
    HasSrc_ = true;
    return *this;
}

TInstruction& TInstruction::SetDst(TTarget target) {
    Dst_ = target;
    HasDst_ = true;
    return *this;
}

TInstruction& TInstruction::SetSize(ESize size) {
    Size_ = size;
    return *this;
}

TInstruction& TInstruction::SetData(TWord data) {
    Data_ = data;
    return *this;
}

std::optional<TError> TInstruction::Execute(NEmulator::TContext ctx) {

#define SAFE_CALL(arg)                              \
    if (auto err = arg) { return std::move(err); }

#define SAFE_DECLARE(name, init)                    \
    const auto name = init;                         \
    if (!name) { return name.error(); }

    switch (Kind_) {
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
        case AddKind:
        case AddiKind: {
            SAFE_DECLARE(srcVal, Src_.ReadAsLongLong(ctx, Size_));
            SAFE_DECLARE(dstVal, Dst_.ReadAsLongLong(ctx, Size_));
            const TLongLong result = *srcVal + *dstVal;
            SAFE_CALL(Dst_.WriteSized(ctx, result, Size_));

            const bool carry = IsCarry(result, Size_);
            ctx.Registers.SetNegativeFlag(GetMsb(result, Size_));
            ctx.Registers.SetCarryFlag(carry);
            ctx.Registers.SetExtendFlag(carry);
            ctx.Registers.SetOverflowFlag(IsOverflow(*srcVal, *dstVal, result, Size_));
            ctx.Registers.SetZeroFlag(IsZero(result, Size_));
            break;
        }
        case AddaKind: {
            TLongLong src;
            if (Size_ == Word) {
                SAFE_DECLARE(srcVal, Src_.ReadWord(ctx));
                src = static_cast<TSignedLongLong>(static_cast<TSignedWord>(*srcVal));
            } else {
                SAFE_DECLARE(srcVal, Src_.ReadLong(ctx));
                src = *srcVal;
            }
            SAFE_DECLARE(dstVal, Dst_.ReadLong(ctx));
            const TLongLong result = src + *dstVal;
            SAFE_CALL(Dst_.WriteSized(ctx, result, Long));
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
        case AddxKind: {
            SAFE_DECLARE(srcVal, Src_.ReadAsLongLong(ctx, Size_));
            SAFE_DECLARE(dstVal, Dst_.ReadAsLongLong(ctx, Size_));
            const TLongLong result = *srcVal + *dstVal + ctx.Registers.GetExtendFlag();
            SAFE_CALL(Dst_.WriteSized(ctx, result, Size_));

            const bool carry = IsCarry(result, Size_);
            ctx.Registers.SetNegativeFlag(GetMsb(result, Size_));
            ctx.Registers.SetCarryFlag(carry);
            ctx.Registers.SetExtendFlag(carry);
            ctx.Registers.SetOverflowFlag(IsOverflow(*srcVal, *dstVal, result, Size_));
            if (!IsZero(result, Size_)) {
                ctx.Registers.SetZeroFlag(0);
            }
            break;
        }
        case AndKind:
        case AndiKind: {
            SAFE_DECLARE(srcVal, Src_.ReadAsLongLong(ctx, Size_));
            SAFE_DECLARE(dstVal, Dst_.ReadAsLongLong(ctx, Size_));
            const TLongLong result = *srcVal & *dstVal;
            SAFE_CALL(Dst_.WriteSized(ctx, result, Size_));

            ctx.Registers.SetNegativeFlag(GetMsb(result, Size_));
            ctx.Registers.SetZeroFlag(IsZero(result, Size_));
            ctx.Registers.SetOverflowFlag(0);
            ctx.Registers.SetCarryFlag(0);
            break;
        }
        case AndiToCcrKind: {
            SAFE_DECLARE(srcVal, Src_.ReadByte(ctx));
            auto& sr = ctx.Registers.SR;
            sr = (sr & ~0xFF) | ((sr & 0xFF) & *srcVal);
            break;
        }
        case AndiToSrKind: {
            SAFE_DECLARE(srcVal, Src_.ReadWord(ctx));
            ctx.Registers.SR &= *srcVal;
            break;
        }
        case AslKind:
        case AsrKind: {
            SAFE_DECLARE(dstVal, Dst_.ReadAsLongLong(ctx, Size_));

            uint8_t rotation;
            if (HasSrc_) {
                SAFE_DECLARE(srcVal, Src_.ReadAsLongLong(ctx, Size_));
                rotation = *srcVal % 64;
            } else {
                rotation = Data_ ? Data_ : 8;
            }

            TLongLong result = *dstVal;
            bool hasOverflow = false;
            bool curMsb = GetMsb(result, Size_);
            bool lastBitShifted;
            for (int i = 0; i < rotation; ++i) {
                if (Kind_ == AslKind) {
                    lastBitShifted = GetMsb(result, Size_);
                    result <<= 1;
                } else {
                    if (i >= BitCount(Size_)) {
                        lastBitShifted = 0;
                    } else {
                        lastBitShifted = result & 1;
                    }
                    // preserve the most significant bit
                    result = (result >> 1) | (result & (1LL << (BitCount(Size_) - 1)));
                }
                bool newMsb = GetMsb(result, Size_);
                if (curMsb != newMsb) {
                    hasOverflow = true;
                }
                curMsb = newMsb;
            }

            SAFE_CALL(Dst_.WriteSized(ctx, result, Size_));

            ctx.Registers.SetNegativeFlag(GetMsb(result, Size_));
            ctx.Registers.SetZeroFlag(IsZero(result, Size_));
            ctx.Registers.SetOverflowFlag(hasOverflow);
            if (rotation == 0) {
                ctx.Registers.SetCarryFlag(0);
            } else {
                ctx.Registers.SetExtendFlag(lastBitShifted);
                ctx.Registers.SetCarryFlag(lastBitShifted);
            }
            break;
        }
        case NopKind: {
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

    // 00 -> byte, 01 -> word, 02 -> long
    const auto getSize0 = [&getBits]() {
        switch (getBits(6, 2)) {
            case 0: return Byte;
            case 1: return Word;
            case 2: return Long;
            default: __builtin_unreachable();
        }
    };

    const auto parseTargetWithSize = [&](ESize size) -> tl::expected<TTarget, TError> {
        TTarget target;

        const auto mode = getBits(3, 3);
        const auto xn = getBits(0, 3);

        switch (mode) {
            case 0: {
                target.SetKind(TTarget::DataRegisterKind).SetIndex(xn);
                break;
            }
            case 1: {
                target.SetKind(TTarget::AddressRegisterKind).SetIndex(xn);
                break;
            }
            case 2: {
                target.SetKind(TTarget::AddressKind).SetIndex(xn);
                break;
            }
            case 3: {
                target.SetKind(TTarget::AddressIncrementKind).SetIndex(xn).SetSize(size);
                break;
            }
            case 4: {
                target.SetKind(TTarget::AddressDecrementKind).SetIndex(xn).SetSize(size);
                break;
            }
            case 5: {
                READ_WORD_SAFE;
                target.SetKind(TTarget::AddressDisplacementKind).SetIndex(xn).SetExtWord0(*word);
                break;
            }
            case 6: {
                READ_WORD_SAFE;
                target.SetKind(TTarget::AddressIndexKind).SetIndex(xn).SetExtWord0(*word);
                break;
            }
            case 7: {
                switch (xn) {
                    case 0: {
                        READ_WORD_SAFE;
                        target.SetKind(TTarget::AbsoluteShortKind).SetExtWord0(*word);
                        break;
                    }
                    case 1: {
                        READ_TWO_WORDS_SAFE;
                        target.SetKind(TTarget::AbsoluteLongKind).SetExtWord0(*word0).SetExtWord1(*word1);
                        break;
                    }
                    case 2: {
                        READ_WORD_SAFE;
                        target.SetKind(TTarget::ProgramCounterDisplacementKind).SetExtWord0(*word);
                        break;
                    }
                    case 3: {
                        READ_WORD_SAFE;
                        target.SetKind(TTarget::ProgramCounterIndexKind).SetExtWord0(*word);
                        break;
                    }
                    case 4: {
                        auto& pc = ctx.Registers.PC;
                        target.SetKind(TTarget::ImmediateKind).SetAddress((size == Byte) ? (pc + 1) : pc);
                        pc += (size == Long) ? 4 : 2;
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

        return target;
    };

    const auto parseTarget = [&]() {
        return parseTargetWithSize(getSize0());
    };

    // decode the opcode
    TInstruction inst;

    if (*word == 0b0100'1100'0111'0001) {
        inst.SetKind(NopKind);
    }
    if (*word == 0b0000'0010'0011'1100) {
        auto& pc = ctx.Registers.PC;
        auto src = TTarget{}.SetKind(TTarget::ImmediateKind).SetAddress(pc + 1);
        pc += 2;
        inst.SetKind(AndiToCcrKind).SetSrc(src);
    }
    if (*word == 0b0000'0010'0111'1100) {
        auto& pc = ctx.Registers.PC;
        auto src = TTarget{}.SetKind(TTarget::ImmediateKind).SetAddress(pc);
        pc += 2;
        inst.SetKind(AndiToSrKind).SetSrc(src);
    }
    else if (applyMask(0b1111'1111'0000'0000) == 0b0000'0110'0000'0000) {
        auto& pc = ctx.Registers.PC;
        auto src = TTarget{}.SetKind(TTarget::ImmediateKind).SetAddress((getSize0() == Byte) ? (pc + 1) : pc);
        pc += (getSize0() == Long) ? 4 : 2;

        const auto dst = parseTarget();
        if (!dst) { return tl::unexpected(dst.error()); }
        inst.SetKind(AddiKind).SetSrc(src).SetDst(*dst).SetSize(getSize0());
    }
    else if (applyMask(0b1111'1111'0000'0000) == 0b0000'0010'0000'0000) {
        auto& pc = ctx.Registers.PC;
        auto src = TTarget{}.SetKind(TTarget::ImmediateKind).SetAddress((getSize0() == Byte) ? (pc + 1) : pc);
        pc += (getSize0() == Long) ? 4 : 2;

        const auto dst = parseTarget();
        if (!dst) { return tl::unexpected(dst.error()); }
        inst.SetKind(AndiKind).SetSrc(src).SetDst(*dst).SetSize(getSize0());
    }
    else if (applyMask(0b1111'0001'0000'0000) == 0b0101'0000'0000'0000) {
        const auto dst = parseTarget();
        if (!dst) { return tl::unexpected(dst.error()); }
        inst.SetKind(AddqKind).SetData(getBits(9, 3)).SetDst(*dst).SetSize(getSize0());
    }
    else if (applyMask(0b1111'0001'1111'0000) == 0b1100'0001'0000'0000) {
        const auto kind = getBit(3) ? TTarget::AddressDecrementKind : TTarget::DataRegisterKind;
        auto src = TTarget{}.SetKind(kind).SetIndex(getBits(0, 3)).SetSize(1);
        auto dst = TTarget{}.SetKind(kind).SetIndex(getBits(9, 3)).SetSize(1);
        inst.SetKind(AbcdKind).SetSrc(src).SetDst(dst);
    }
    else if (applyMask(0b1111'0000'1100'0000) == 0b1101'0000'1100'0000) {
        const auto size = getBit(8) ? Long : Word;

        auto src = parseTargetWithSize(size);
        if (!src) { return tl::unexpected(src.error()); }

        auto dst = TTarget{}.SetKind(TTarget::AddressRegisterKind).SetIndex(getBits(9, 3));

        inst.SetKind(AddaKind).SetSrc(*src).SetDst(dst).SetSize(size);
    }
    else if (applyMask(0b1111'0001'0011'0000) == 0b1101'0001'0000'0000) {
        const auto size = getSize0();
        const auto kind = getBit(3) ? TTarget::AddressDecrementKind : TTarget::DataRegisterKind;
        auto src = TTarget{}.SetKind(kind).SetIndex(getBits(0, 3)).SetSize(size);
        auto dst = TTarget{}.SetKind(kind).SetIndex(getBits(9, 3)).SetSize(size);
        inst.SetKind(AddxKind).SetSrc(src).SetDst(dst).SetSize(size);
    }
    else if (applyMask(0b1111'0000'0000'0000) == 0b1100'0000'0000'0000) {
        auto src = TTarget{}.SetKind(TTarget::DataRegisterKind).SetIndex(getBits(9, 3));

        auto dst = parseTarget();
        if (!dst) { return tl::unexpected(dst.error()); }

        if (!getBit(8)) {
            std::swap(src, *dst);
        }

        inst.SetKind(AndKind).SetSrc(src).SetDst(*dst).SetSize(getSize0());
    }
    else if (applyMask(0b1111'0000'0000'0000) == 0b1101'0000'0000'0000) {
        auto src = TTarget{}.SetKind(TTarget::DataRegisterKind).SetIndex(getBits(9, 3));

        auto dst = parseTarget();
        if (!dst) { return tl::unexpected(dst.error()); }

        if (!getBit(8)) {
            std::swap(src, *dst);
        }

        inst.SetKind(AddKind).SetSrc(src).SetDst(*dst).SetSize(getSize0());
    }
    else if (applyMask(0b1111'1110'1100'0000) == 0b1110'0000'1100'0000) {
        // ASL/ASR on any memory, shift by 1
        auto kind = getBit(8) ? AslKind : AsrKind;
        auto dst = parseTargetWithSize(Word);
        if (!dst) { return tl::unexpected(dst.error()); }

        inst.SetKind(kind).SetDst(*dst).SetSize(Word).SetData(1);
    }
    else if (applyMask(0b1111'0000'0001'1000) == 0b1110'0000'0000'0000) {
        // ASL/ASR on Dn
        auto kind = getBit(8) ? AslKind : AsrKind;
        uint8_t rotation = getBits(9, 3);
        auto dst = TTarget{}.SetKind(TTarget::DataRegisterKind).SetIndex(getBits(0, 3));

        inst.SetKind(kind).SetDst(dst).SetSize(getSize0());
        if (getBit(5)) {
            // shift count is in the data register
            auto src = TTarget{}.SetKind(TTarget::DataRegisterKind).SetIndex(rotation);
            inst.SetSrc(src);
        } else {
            // shift count is immediate
            inst.SetData(rotation);
        }
    }
    else {
        return tl::unexpected<TError>(TError::UnknownOpcode, "Unknown opcode %#04x", *word);
    }

    return inst;
}

} // namespace NOpcodes
