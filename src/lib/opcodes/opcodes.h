#pragma once

#include <variant>

namespace NOpcodes {

namespace NList {

struct NOP {};

} // namespace NList

using TOneOf = std::variant<NList::NOP>;

} // namespace NOpcodes
