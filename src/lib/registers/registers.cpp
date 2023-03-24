#include "registers.h"

#include <sstream>

namespace NRegisters {

namespace {

void SetFlag(bool flag, TWord& r, int index) {
    if (flag) {
        r |= 1 << index;
    } else {
        r &= r ^ (1 << index);
    }
}

bool GetFlag(TWord r, int index) {
    return r & (1 << index);
}

} // namespace

void TRegisters::SetSupervisorFlag(bool flag) { return SetFlag(flag, SR, 13); }
bool TRegisters::GetSupervisorFlag() const { return GetFlag(SR, 13); }

void TRegisters::SetExtendFlag(bool flag) { return SetFlag(flag, SR, 4); }
bool TRegisters::GetExtendFlag() const { return GetFlag(SR, 4); }

void TRegisters::SetNegativeFlag(bool flag) { return SetFlag(flag, SR, 3); }
bool TRegisters::GetNegativeFlag() const { return GetFlag(SR, 3); }

void TRegisters::SetZeroFlag(bool flag) { return SetFlag(flag, SR, 2); }
bool TRegisters::GetZeroFlag() const { return GetFlag(SR, 2); }

void TRegisters::SetOverflowFlag(bool flag) { return SetFlag(flag, SR, 1); }
bool TRegisters::GetOverflowFlag() const { return GetFlag(SR, 1); }

void TRegisters::SetCarryFlag(bool flag) { return SetFlag(flag, SR, 0); }
bool TRegisters::GetCarryFlag() const { return GetFlag(SR, 0); }

std::string Dump(const TRegisters& r) {
    std::stringstream ss;
    ss << std::hex << std::uppercase;
    for (int i = 0; i < 7; ++i) {
        ss << "D" << i << " = " << r.D[i] << "\tA" << i << " = " << r.A[i] << "\n";
    }
    ss << "D7 = " << r.D[7] << "\n";
    ss << "USP = " << r.USP << "\n";
    ss << "SSP = " << r.SSP << "\n";
    ss << "PC = " << r.PC << "\n";

    ss << "SR: ";
    ss << "T = " << ((r.SR >> 14) & 3) << ", ";
    ss << "S = " << ((r.SR >> 13) & 1) << ", ";
    ss << "M = " << ((r.SR >> 12) & 1) << ", ";
    ss << "I = " << ((r.SR >> 8) & 7) << ", ";
    ss << "X = " << ((r.SR >> 4) & 1) << ", ";
    ss << "N = " << ((r.SR >> 3) & 1) << ", ";
    ss << "Z = " << ((r.SR >> 2) & 1) << ", ";
    ss << "V = " << ((r.SR >> 1) & 1) << ", ";
    ss << "C = " << (r.SR & 1) << "\n";

    return ss.str();
}

} // namespace NRegisters
