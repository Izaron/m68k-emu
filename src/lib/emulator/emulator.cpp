#include "emulator.h"

#include <lib/opcodes/instructions.h>

namespace NEmulator {

std::optional<TError> Emulate(TContext ctx) {
    auto inst = NOpcodes::TInstruction::Decode(ctx);
    if (!inst) {
        return inst.error();
    }
    return inst->Execute(ctx);
}

} // namespace NEmulator
