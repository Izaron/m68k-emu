#include "instructions.h"

#include <stdexcept>
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

} // namespace

void TInstruction::SetNop() {
    Kind_ = NopKind;
}

void TInstruction::SetAbcd(TTarget src, TTarget dst) {
    Kind_ = AbcdKind;
    Value_.AbcdData.Src = src;
    Value_.AbcdData.Dst = dst;
}

void TInstruction::SetAdd(TTarget src, TTarget dst, ESize size) {
    Kind_ = AddKind;
    Value_.AddData.Src = src;
    Value_.AddData.Dst = dst;
    Value_.AddData.Size = size;
}

void TInstruction::Execute(NEmulator::TContext ctx) {
    switch (Kind_) {
        case NopKind: {
            break;
        }
        case AbcdKind: {
            auto& data = Value_.AbcdData;
            data.Src.PreWork(ctx);
            data.Dst.PreWork(ctx);

            const TByte srcVal = data.Src.ReadByte(ctx);
            const TByte dstVal = data.Dst.ReadByte(ctx);
            const TByte extendFlag = ctx.Registers.GetExtendFlag();

            const TWord binaryResult = srcVal + dstVal + extendFlag;

            bool carry = false;
            int lval = (srcVal & 0x0F) + (dstVal & 0x0F) + extendFlag;
            if (lval > 9) {
                carry = true;
                lval -= 10;
            }

            int hval = ((srcVal >> 4) & 0x0F) + ((dstVal >> 4) & 0x0F) + (carry ? 1 : 0);
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

            data.Dst.WriteByte(ctx, result);

            ctx.Registers.SetNegativeFlag(GetMostSignificantBit<TByte>(result));
            ctx.Registers.SetCarryFlag(carry);
            ctx.Registers.SetExtendFlag(carry);
            ctx.Registers.SetOverflowFlag((~binaryResult & result & 0x80) != 0);
            if (result != 0) {
                ctx.Registers.SetZeroFlag(result == 0);
            }

            data.Src.PostWork(ctx);
            data.Dst.PostWork(ctx);

            break;
        }
        case AddKind: {
            auto& data = Value_.AbcdData;
            data.Src.PreWork(ctx);
            data.Dst.PreWork(ctx);

            const TByte srcVal = data.Src.ReadByte(ctx);
            const TByte dstVal = data.Dst.ReadByte(ctx);

            const TWord result = TWord{0} + srcVal + dstVal;
            const bool carry = result & ~0xFF;

            const bool srcMsb = GetMostSignificantBit<TByte>(srcVal);
            const bool dstMsb = GetMostSignificantBit<TByte>(dstVal);
            const bool resultMsb = GetMostSignificantBit<TByte>(result);
            const bool overflow = (srcMsb && dstMsb && !resultMsb) || (!srcMsb && !dstMsb && resultMsb);

            data.Dst.WriteByte(ctx, result);
            ctx.Registers.SetNegativeFlag(resultMsb);
            ctx.Registers.SetCarryFlag(carry);
            ctx.Registers.SetExtendFlag(carry);
            ctx.Registers.SetOverflowFlag(overflow);
            ctx.Registers.SetZeroFlag((result & 0xFF) == 0);

            data.Src.PostWork(ctx);
            data.Dst.PostWork(ctx);
            break;
        }
    }
}

TInstruction TInstruction::Decode(NEmulator::TContext ctx) {
    const auto readWord = [&ctx]() {
        auto& pc = ctx.Registers.PC;
        const TWord word = ctx.Memory.ReadWord(pc);
        pc += 2;
        return word;
    };

    // read the current word (16 bits)
    const TWord word = readWord();

    // helper functions
    const auto applyMask = [word](TWord mask) { return word & mask; };
    const auto getBits = [word](std::size_t begin, std::size_t len) { return (word >> begin) & ((1 << len) - 1); };
    const auto getBit = [word, getBits](std::size_t bit) { return getBits(bit, 1); };

    const auto getSize0 = [word, getBits](std::size_t bit) {
        // 00 -> byte, 01 -> word, 02 -> long
        switch (getBits(bit, 2)) {
            case 0: return Byte;
            case 1: return Word;
            case 2: return Long;
            default: __builtin_unreachable();
        }
    };

    // decode the opcode
    TInstruction inst;

    if (word == 0b0100'1100'0111'0001) {
        inst.SetNop();
    }
    else if (applyMask(0b1111'0001'1111'0000) == 0b1100'0001'0000'0000) {
        const auto func = getBit(3) ? &TTarget::SetAddressDecrement : &TTarget::SetDataRegister;
        TTarget src;
        (src.*func)(getBits(0, 3));
        TTarget dst;
        (dst.*func)(getBits(9, 3));
        inst.SetAbcd(src, dst);
    }
    else if (applyMask(0b1111'0000'0000'0000) == 0b1101'0000'0000'0000) {
        TTarget src;
        src.SetDataRegister(getBits(9, 3));

        TTarget dst;

        const ESize size = getSize0(6);
        const auto mode = getBits(3, 3);
        const auto xn = getBits(0, 3);

        switch (mode) {
            case 0: {
                dst.SetDataRegister(xn);
                break;
            }
            case 1: {
                dst.SetAddressRegister(xn);
                break;
            }
            case 2: {
                dst.SetAddress(xn);
                break;
            }
            case 3: {
                dst.SetAddressIncrement(xn);
                break;
            }
            case 4: {
                dst.SetAddressDecrement(xn);
                break;
            }
            case 5: {
                dst.SetAddressDisplacement(xn, readWord());
                break;
            }
            case 6: {
                dst.SetAddressIndex(xn, readWord());
                break;
            }
            case 7: {
                switch (xn) {
                    case 0: {
                        dst.SetAbsoluteShort(readWord());
                        break;
                    }
                    case 1: {
                        const TWord extWord0 = readWord();
                        const TWord extWord1 = readWord();
                        dst.SetAbsoluteLong(extWord0, extWord1);
                        break;
                    }
                    case 2: {
                        dst.SetProgramCounterDisplacement(readWord());
                        break;
                    }
                    case 3: {
                        dst.SetProgramCounterIndex(readWord());
                        break;
                    }
                    case 4: {
                        auto& pc = ctx.Registers.PC;
                        dst.SetImmediate((size == Byte) ? (pc + 1) : pc);
                        pc += (size == Long) ? 4 : 2;
                        break;
                    }
                    default: {
                        dst.SetDataRegister(0);
                        break;
                    }
                }
                break;
            }
            default: {
                dst.SetDataRegister(0);
                break;
            }
        }

        if (!getBit(8)) {
            std::swap(src, dst);
        }

        inst.SetAdd(src, dst, size);
    }
    else {
        // TODO: place runtime error
        std::cerr << "Unknown code, use NOP " << word << std::endl;
        throw std::runtime_error("Unknown code");
        inst.SetNop();
    }

    return inst;
}

} // namespace NOpcodes
