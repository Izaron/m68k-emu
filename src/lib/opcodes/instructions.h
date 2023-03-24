#pragma once

#include <lib/emulator/emulator.h>

#include "targets.h"

namespace NOpcodes {

class TInstruction {
public:
    enum EKind : uint8_t {
        AbcdKind,   // ABCD
        AddKind,    // ADD
        AddiKind,   // ADDI
        AddqKind,   // ADDI
        NopKind,    // NOP
    };

    enum ESize : uint8_t {
        Byte = 1,
        Word = 2,
        Long = 4,
    };

    void SetAbcd(TTarget src, TTarget dst);
    void SetAdd(TTarget src, TTarget dst, ESize size);
    void SetAddi(TTarget src, TTarget dst, ESize size);
    void SetAddq(TWord data, TTarget dst, ESize size);
    void SetNop();

    [[nodiscard]] std::optional<TError> Execute(NEmulator::TContext ctx);

    static tl::expected<TInstruction, TError> Decode(NEmulator::TContext ctx);

private:
    EKind Kind_;
    TWord Data_;
    TTarget Src_;
    TTarget Dst_;
    ESize Size_;

    bool HasSrc_;
    bool HasDst_;
};

} // namespace NOpcodes
