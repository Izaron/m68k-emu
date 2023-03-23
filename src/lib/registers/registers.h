#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace NRegisters {

/**
 * Registers are signed 32-bit integers
 */
using TRegisterType = int32_t;

struct TRegisters {
    /**
     * Data registers D0 - D7
     */
    std::array<TRegisterType, 8> D;

    /**
     * Address registers A0 - A6
     */
    std::array<TRegisterType, 7> A;

    /**
     * User stack pointer
     */
    TRegisterType USP;

    /**
     * Supervisor stack pointer
     */
    TRegisterType SSP;

    /**
     * Program counter
     */
    TRegisterType PC;

    /**
     * Status register
     * TODO: add description for each bit?
     */
    TRegisterType SR;

    /**
     * Status register helpers
     */
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
