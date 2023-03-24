#pragma once

#include <lib/error/error.h>
#include <lib/memory/i_device.h>
#include <lib/registers/registers.h>

namespace NEmulator {

struct TContext {
    NRegisters::TRegisters& Registers;
    NMemory::IDevice& Memory;
};

std::optional<TError> Emulate(TContext ctx);

} // namespace NEmulator
