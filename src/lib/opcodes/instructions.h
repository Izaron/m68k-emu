#pragma once

#include <lib/emulator/emulator.h>

#include "targets.h"

namespace NOpcodes {

class TInstruction {
public:
    enum EKind : uint8_t {
        AbcdKind,       // ABCD
        AddKind,        // ADD
        AddaKind,       // ADDA
        AddiKind,       // ADDI
        AddqKind,       // ADDQ
        AddxKind,       // ADDX
        AndKind,        // AND
        AndiKind,       // ANDI
        AndiToCcrKind,  // ANDItoCCR
        AndiToSrKind,   // ANDItoSR
        AslKind,        // ASL
        AsrKind,        // ASR
        BccKind,        // Bcc
        NopKind,        // NOP
    };

    enum ESize : uint8_t {
        Byte = 1,
        Word = 2,
        Long = 4,
    };

    enum ECondition : uint8_t {
        TrueCond,               // T
        FalseCond,              // F
        HigherCond,             // HI
        LowerOrSameCond,        // LS
        CarryClearCond,         // CC
        CarrySetCond,           // CS
        NotEqualCond,           // NE
        EqualCond,              // EQ
        OverflowClearCond,      // VC
        OverflowSetCond,        // VS
        PlusCond,               // PL
        MinusCond,              // MI
        GreaterOrEqualCond,     // GE
        LessThanCond,           // LT
        GreaterThanCond,        // GT
        LessOrEqualCond,        // LE
    };

    TInstruction& SetKind(EKind kind);
    TInstruction& SetSize(ESize size);
    TInstruction& SetCondition(ECondition cond);
    TInstruction& SetSrc(TTarget target);
    TInstruction& SetDst(TTarget target);
    TInstruction& SetData(TWord data);

    [[nodiscard]] std::optional<TError> Execute(NEmulator::TContext ctx);

    static tl::expected<TInstruction, TError> Decode(NEmulator::TContext ctx);

private:
    EKind Kind_;
    ESize Size_;
    ECondition Cond_;
    TTarget Src_;
    TTarget Dst_;
    TWord Data_;

    bool HasSrc_;
    bool HasDst_;
};

} // namespace NOpcodes
