#pragma once

#include <lib/memory/i_device.h>
#include <lib/registers/registers.h>

namespace NEmulator {

struct TContext {
    NRegisters::TRegisters& Registers;
    NMemory::IDevice& Memory;
};

void Emulate(TContext ctx);

} // namespace NEmulator
