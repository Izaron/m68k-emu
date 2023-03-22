#include "emulator.h"

#include <lib/opcodes/opcodes.h>

namespace NEmulator {

namespace {

template<typename T> T ToValue(NMemory::TDataView data) {
    T res = 0;
    for (auto& b : data) {
        res = (res << 8) + static_cast<T>(b);
    }
    return res;
}

NOpcodes::TOneOf DecodeOpcode() {
    return NOpcodes::NList::NOP{};
}

class TVisitor {
public:
    void operator()(const NOpcodes::NList::NOP&) {
        // no op!
    }
};

} // namespace

void Emulate(TContext ctx) {
    // read two bytes (16 bits)
    auto& pc = ctx.Registers.PC;
    auto data = ctx.Memory.Read(pc, 2);
    pc += 2;

    // decode the opcode
    auto word = ToValue<uint16_t>(data);
    auto opcode = DecodeOpcode();

    TVisitor visitor;
    std::visit(visitor, opcode);
}

} // namespace NEmulator
