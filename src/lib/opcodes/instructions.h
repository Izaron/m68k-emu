#pragma once

#include <lib/emulator/emulator.h>

#include "targets.h"

namespace NOpcodes {

class TInstruction {
public:
    enum EKind : uint8_t {
        NopKind,    // NOP
        AbcdKind,   // ABCD
    };

    void SetNop();
    void SetAbcd(TTarget src, TTarget dst);

    void Execute(NEmulator::TContext ctx);

    static TInstruction Decode(NEmulator::TContext ctx);

private:
    union {
        struct {
            TTarget Src;
            TTarget Dst;
        } AbcdData;
    } Value_;

    EKind Kind_;
};

} // namespace NOpcodes
