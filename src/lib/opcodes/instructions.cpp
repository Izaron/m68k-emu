#include "instructions.h"

#include <type_traits>

namespace NOpcodes {

static_assert(std::is_trivially_constructible_v<TInstruction>);

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
            TWord result = (srcVal & 0x0F) + (dstVal & 0x0F) + extendFlag;
            if (result > 0x09) {
                result += 0x06;
            }
            result += (srcVal & 0xF0) + (dstVal & 0xF0);
            if (result > 0x99) {
                result += 0x60;
            }

            data.Dst.WriteByte(ctx, result);
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
        const auto func = getBit(3) ? &TTarget::SetDecAddressRegisterKind : &TTarget::SetDataRegister;
        TTarget src;
        (src.*func)(getBits(0, 3));
        TTarget dst;
        (dst.*func)(getBits(9, 3));
        inst.SetAbcd(src, dst);
    }

    return inst;
}

} // namespace NOpcodes
