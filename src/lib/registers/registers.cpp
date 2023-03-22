#include "registers.h"

#include <sstream>

namespace NRegisters {

std::string Dump(const TRegisters& r) {
    std::stringstream ss;
    //ss << std::hex << std::uppercase;
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
