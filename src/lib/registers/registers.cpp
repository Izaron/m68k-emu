#include "registers.h"

#include <sstream>

namespace NRegisters {

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
    ss << "SR = " << r.SR << "\n";
    return ss.str();
}

} // namespace NRegisters
