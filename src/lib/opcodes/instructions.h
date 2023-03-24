#pragma once

#include <lib/emulator/emulator.h>

#include "targets.h"

namespace NOpcodes {

class TInstruction {
public:
    enum EKind : uint8_t {
        AbcdKind,   // ABCD
        AddKind,    // ADD
        AddaKind,   // ADDA
        AddiKind,   // ADDI
        AddqKind,   // ADDQ
        AddxKind,   // ADDX
        AndKind,    // AND
        AndiKind,   // ANDI
        NopKind,    // NOP
    };

    enum ESize : uint8_t {
        Byte = 1,
        Word = 2,
        Long = 4,
    };

    TInstruction& SetKind(EKind kind);
    TInstruction& SetSrc(TTarget target);
    TInstruction& SetDst(TTarget target);
    TInstruction& SetSize(ESize size);
    TInstruction& SetData(TWord data);

    [[nodiscard]] std::optional<TError> Execute(NEmulator::TContext ctx);

    static tl::expected<TInstruction, TError> Decode(NEmulator::TContext ctx);

private:
    EKind Kind_;
    TTarget Src_;
    TTarget Dst_;
    ESize Size_;
    TWord Data_;

    bool HasSrc_;
    bool HasDst_;
};

} // namespace NOpcodes
