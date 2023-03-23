#pragma once

#include <cstdint>
#include <variant>

namespace NOpcodes {

/**
 * Targets
 */
namespace NTargetList {

    using TRegisterIndex = uint8_t;

    struct DataRegister { TRegisterIndex Index; };
    struct DecrementAddressRegister { TRegisterIndex Index; };

    using TOneOf = std::variant<
        DataRegister, DecrementAddressRegister
    >;

} // namespace NTargetList

using TTargetOneOf = NTargetList::TOneOf;

/**
 * Instructions
 */
namespace NInstructionList {

    struct NOP {};
    struct ABCD { TTargetOneOf Src; TTargetOneOf Dst; };

    using TOneOf = std::variant<
        NOP, ABCD
    >;

} // namespace NInstructionList

using TInstructionOneOf = NInstructionList::TOneOf;

} // namespace NOpcodes
