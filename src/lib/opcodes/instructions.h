#pragma once

#include <lib/emulator/emulator.h>

#include "targets.h"

namespace NOpcodes {

class TInstruction {
public:
    enum EKind : uint8_t {
        AbcdKind,           // ABCD
        AddKind,            // ADD
        AddaKind,           // ADDA
        AddiKind,           // ADDI
        AddqKind,           // ADDQ
        AddxKind,           // ADDX
        AndKind,            // AND
        AndiKind,           // ANDI
        AndiToCcrKind,      // ANDItoCCR
        AndiToSrKind,       // ANDItoSR
        AslKind,            // ASL
        AsrKind,            // ASR
        BccKind,            // Bcc
        BchgKind,           // BCHG
        BclrKind,           // BCLR
        BsetKind,           // BSET
        BsrKind,            // BSR
        BtstKind,           // BTST
        ClrKind,            // CLR
        CmpKind,            // CMP
        CmpaKind,           // CMPA
        CmpiKind,           // CMPI
        CmpmKind,           // CMPM
        EorKind,            // EOR
        EoriKind,           // EORI
        EoriToCcrKind,      // EORItoCCR
        EoriToSrKind,       // EORItoSR
        LslKind,            // LSL
        LsrKind,            // LSR
        MoveFromSrKind,     // MOVEfromSR
        MoveFromUspKind,    // MOVEfromUSP
        MoveKind,           // MOVE
        MoveToCcrKind,      // MOVEtoCCR
        MoveToSrKind,       // MOVEtoSR
        MoveaKind,          // MOVEA
        MoveqKind,          // MOVEQ
        NegKind,            // NEG
        NegxKind,           // NEGX
        NopKind,            // NOP
        NotKind,            // NOT
        OrKind,             // OR
        OriKind,            // ORI
        OriToCcrKind,       // ORItoCCR
        OriToSrKind,        // ORItoSR
        SubKind,            // SUB
        SubaKind,           // SUBA
        SubiKind,           // SUBI
        SubqKind,           // SUBQ
        SubxKind,           // SUBX
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
