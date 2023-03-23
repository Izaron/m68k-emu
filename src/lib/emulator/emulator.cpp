#include "emulator.h"

#include <lib/opcodes/instructions.h>

namespace NEmulator {

void Emulate(TContext ctx) {
    auto inst = NOpcodes::TInstruction::Decode(ctx);
    inst.Execute(ctx);
}

} // namespace NEmulator
