#include "instructions.h"

#include <type_traits>

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

void TInstruction::Execute(NEmulator::TContext ctx) {
    switch (Kind_) {
        case NopKind: {
            break;
        }
        case AbcdKind: {
            auto& data = Value_.AbcdData;
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
            break;
        }
    }
}

TInstruction TInstruction::Decode(NEmulator::TContext ctx) {
    // read the current word (16 bits)
    auto& pc = ctx.Registers.PC;
    const TWord word = ctx.Memory.ReadWord(pc);
    pc += 2;

    // helper functions
    const auto applyMask = [word](TWord mask) { return word & mask; };
    const auto getBits = [word](std::size_t begin, std::size_t len) { return (word >> begin) & ((1 << len) - 1); };
    const auto getBit = [word, getBits](std::size_t bit) { return getBits(bit, 1); };

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

    return inst;
}

} // namespace NOpcodes
