#pragma once

#include <lib/emulator/emulator.h>

#include "targets.h"

namespace NOpcodes {

class TInstruction {
public:
    enum EKind : uint8_t {
        NopKind,    // NOP
        AbcdKind,   // ABCD
        AddKind,    // ADD
    };

    enum ESize : uint8_t {
        Byte = 1,
        Word = 2,
        Long = 4,
    };

    void SetNop();
    void SetAbcd(TTarget src, TTarget dst);
    void SetAdd(TTarget src, TTarget dst, ESize size);

    void Execute(NEmulator::TContext ctx);

    static TInstruction Decode(NEmulator::TContext ctx);

private:
    EKind Kind_;
    TTarget Src_;
    TTarget Dst_;
    ESize Size_;

    bool HasSrc_;
    bool HasDst_;
};

} // namespace NOpcodes
