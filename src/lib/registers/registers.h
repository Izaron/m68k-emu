#pragma once

#include <array>
#include <string>

#include <lib/memory/types.h>

namespace NRegisters {

struct TRegisters {
    /**
     * Data registers D0 - D7
     */
    std::array<TLong, 8> D;

    /**
     * Address registers A0 - A6
     */
    std::array<TLong, 7> A;

    /**
     * User stack pointer
     */
    TLong USP;

    /**
     * Supervisor stack pointer
     */
    TLong SSP;

    /**
     * Program counter
     */
    TLong PC;

    /**
     * Status register
     * TODO: add description for each bit?
     */
    TLong SR;

    /**
     * Status register helpers
     */
    void SetSupervisorFlag(bool flag);
    bool GetSupervisorFlag() const;

    void SetExtendFlag(bool flag);
    bool GetExtendFlag() const;

    void SetNegativeFlag(bool flag);
    bool GetNegativeFlag() const;

    void SetZeroFlag(bool flag);
    bool GetZeroFlag() const;

    void SetOverflowFlag(bool flag);
    bool GetOverflowFlag() const;

    void SetCarryFlag(bool flag);
    bool GetCarryFlag() const;
};

std::string Dump(const TRegisters& registers);

} // namespace NRegisters
