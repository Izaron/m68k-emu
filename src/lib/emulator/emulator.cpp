#include "emulator.h"

#include <lib/opcodes/opcodes.h>

using namespace NOpcodes;
namespace NList = NOpcodes::NInstructionList;

namespace NEmulator {

namespace {

template<typename T> T ToValue(NMemory::TDataView data) {
    T res = 0;
    for (auto& b : data) {
        res = (res << 8) + static_cast<T>(b);
    }
    return res;
}

class TTargetDataReader {
public:
    TTargetDataReader(TContext ctx)
        : Ctx_{ctx}
    {}

    std::byte operator()(const NTargetList::DecrementAddressRegister& target) {
        auto& reg = Ctx_.Registers.A[target.Index];
        --reg;
        return Ctx_.Memory.Read(reg, 1)[0];
    }

    std::byte operator()(const NTargetList::DataRegister& target) {
        const auto reg = Ctx_.Registers.D[target.Index];
        return Ctx_.Memory.Read(reg, 1)[0];
    }

private:
    TContext Ctx_;
};

class TOpcodeProcessor {
public:
    TOpcodeProcessor(TContext ctx)
        : Ctx_{ctx}
    {}

    void operator()(const NList::NOP&) {
        // no op
    }

    void operator()(const NList::ABCD& instr) {
        TTargetDataReader reader{Ctx_};
        const uint8_t srcVal = static_cast<uint8_t>(std::visit(reader, instr.Src));
        const uint8_t dstVal = static_cast<uint8_t>(std::visit(reader, instr.Dst));

        const int extendFlag = Ctx_.Registers.GetExtendFlag();
        uint8_t result = (srcVal & 0x0F) + (dstVal & 0x0F) + extendFlag;
        if (result > 0x09) {
            result += 0x06;
        }
        result += (srcVal & 0xF0) + (dstVal & 0xF0);
        if (result > 0x99) {
            result += 0x60;
        }
    }

private:
    TContext Ctx_;
};

class TEmulator {
public:
    TEmulator(TContext ctx)
        : Ctx_{ctx}
    {}

    void Emulate() {
        // read two bytes (16 bits)
        auto& pc = Ctx_.Registers.PC;
        auto data = Ctx_.Memory.Read(pc, 2);
        pc += 2;

        // decode the opcode
        Opcode_ = ToValue<uint16_t>(data);
        auto inst = Decode();

        // process the opcode
        TOpcodeProcessor opcodeProcessor{Ctx_};
        std::visit(opcodeProcessor, inst);
    }

private:
    NOpcodes::TInstructionOneOf Decode() {
        using namespace NOpcodes::NTargetList;

        if (Opcode_ == 0b0100'1100'0111'0001) {
            return NList::NOP{};
        }

        if (ApplyMask(0b1111'0001'1111'0000) == 0b1100'0001'0000'0000) {
            const uint8_t src = GetBits(9, 3);
            const uint8_t dst = GetBits(0, 3);
            if (TestBit(3)) {
                return NList::ABCD{.Src = DecrementAddressRegister{src}, .Dst = DecrementAddressRegister{dst}};
            } else {
                return NList::ABCD{.Src = DataRegister{src}, .Dst = DataRegister{dst}};
            }
        }

        return NList::NOP{};
    }

    uint16_t ApplyMask(uint16_t mask) const {
        return Opcode_ & mask;
    }

    bool TestBit(std::size_t bit) const {
        return Opcode_ & (1 << bit);
    }

    uint16_t GetBits(std::size_t begin, std::size_t len) const {
        return (Opcode_ >> begin) & ((1 << len) - 1);
    }

private:
    TContext Ctx_;
    uint16_t Opcode_;
};

} // namespace

void Emulate(TContext ctx) {
    TEmulator{ctx}.Emulate();
}

} // namespace NEmulator
