#pragma once

#include <variant>

namespace NOpcodes {

namespace NList {

struct NOP {};

using TList = std::variant<NOP>;

} // namespace NList

} // namespace NOpcodes
