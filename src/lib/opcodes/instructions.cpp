#include "instructions.h"

#include <type_traits>
#include <iostream>

namespace NOpcodes {

static_assert(std::is_trivially_constructible_v<TInstruction>);

namespace {

enum EOpcodeType {
    AddType,
    AndType,
    CmpType,
    EorType,
    OrType,
    SubType,
};

EOpcodeType GetOpcodeType(TInstruction::EKind kind) {
    if (kind >= TInstruction::AddKind && kind <= TInstruction::AddxKind) {
        return AddType;
    }
    if (kind >= TInstruction::AndKind && kind <= TInstruction::AndiToSrKind) {
        return AndType;
    }
    if (kind >= TInstruction::CmpKind && kind <= TInstruction::CmpmKind) {
        return CmpType;
    }
    if (kind >= TInstruction::EorKind && kind <= TInstruction::EoriToSrKind) {
        return EorType;
    }
    if (kind >= TInstruction::OrKind && kind <= TInstruction::OriToSrKind) {
        return OrType;
    }
    if (kind >= TInstruction::SubKind && kind <= TInstruction::SubxKind) {
        return SubType;
    }
    __builtin_unreachable();
}

auto DoBinaryOp(EOpcodeType type, auto lhs, auto rhs) {
    switch (type) {
        case AddType: return lhs + rhs;
        case AndType: return lhs & rhs;
        case EorType: return lhs ^ rhs;
        case OrType: return  lhs | rhs;
        case SubType: return rhs - lhs;
        case CmpType: return rhs - lhs;
        default: __builtin_unreachable();
    }
}

bool IsSubstractOp(EOpcodeType type) {
    return type == SubType || type == CmpType;
}

bool IsCarry(TLongLong value, TInstruction::ESize size) {
    switch (size) {
        case TInstruction::Byte: return value & (value ^ 0xFF);
        case TInstruction::Word: return value & (value ^ 0xFFFF);
        case TInstruction::Long: return value & (value ^ 0xFFFFFFFF);
        default: __builtin_unreachable();
    }
}

bool IsZero(TLongLong value, TInstruction::ESize size) {
    switch (size) {
        case TInstruction::Byte: return (value & 0xFF) == 0;
        case TInstruction::Word: return (value & 0xFFFF) == 0;
        case TInstruction::Long: return (value & 0xFFFFFFFF) == 0;
        default: __builtin_unreachable();
    }
}

uint8_t BitCount(TInstruction::ESize size) {
    return size << 3;
}

bool GetMsb(TLongLong value, TInstruction::ESize size) {
    return (value >> (BitCount(size) - 1)) & 1;
}

bool IsOverflow(TLongLong lhs, TLongLong rhs, TLongLong result, TInstruction::ESize size, EOpcodeType type = AddType) {
    const bool lhsMsb = GetMsb(lhs, size) ^ (IsSubstractOp(type) ? 1 : 0);
    const bool rhsMsb = GetMsb(rhs, size);
    const bool resultMsb = GetMsb(result, size);
    return (lhsMsb && rhsMsb && !resultMsb) || (!lhsMsb && !rhsMsb && resultMsb);
}

bool CalculateCondition(const NRegisters::TRegisters& regs, TInstruction::ECondition cond) {
    switch (cond) {
        case TInstruction::TrueCond:
            return true;
        case TInstruction::FalseCond:
            return false;
        case TInstruction::HigherCond:
            return (not regs.GetCarryFlag()) && (not regs.GetZeroFlag());
        case TInstruction::LowerOrSameCond:
            return regs.GetCarryFlag() || regs.GetZeroFlag();
        case TInstruction::CarryClearCond:
            return not regs.GetCarryFlag();
        case TInstruction::CarrySetCond:
            return     regs.GetCarryFlag();
        case TInstruction::NotEqualCond:
            return not regs.GetZeroFlag();
        case TInstruction::EqualCond:
            return     regs.GetZeroFlag();
        case TInstruction::OverflowClearCond:
            return not regs.GetOverflowFlag();
        case TInstruction::OverflowSetCond:
            return     regs.GetOverflowFlag();
        case TInstruction::PlusCond:
            return not regs.GetNegativeFlag();
        case TInstruction::MinusCond:
            return     regs.GetNegativeFlag();
        case TInstruction::GreaterOrEqualCond:
            return not (regs.GetNegativeFlag() xor regs.GetOverflowFlag());
        case TInstruction::LessThanCond:
            return      regs.GetNegativeFlag() xor regs.GetOverflowFlag();
        case TInstruction::GreaterThanCond:
            return (regs.GetNegativeFlag() and regs.GetOverflowFlag() and (not regs.GetZeroFlag())) or
                   ((not regs.GetNegativeFlag()) and (not regs.GetOverflowFlag()) and (not regs.GetZeroFlag()));
        case TInstruction::LessOrEqualCond:
            return regs.GetZeroFlag() or (regs.GetNegativeFlag() and (not regs.GetOverflowFlag())) or
                   ((not regs.GetNegativeFlag()) and regs.GetOverflowFlag());
        default:
            __builtin_unreachable();
    }
}

} // namespace

TInstruction& TInstruction::SetKind(EKind kind) {
    Kind_ = kind;
    HasSrc_ = HasDst_ = false;
    return *this;
}

TInstruction& TInstruction::SetSize(ESize size) {
    Size_ = size;
    return *this;
}

TInstruction& TInstruction::SetCondition(ECondition cond) {
    Cond_ = cond;
    return *this;
}

TInstruction& TInstruction::SetSrc(TTarget target) {
    Src_ = target;
    HasSrc_ = true;
    return *this;
}

TInstruction& TInstruction::SetDst(TTarget target) {
    Dst_ = target;
    HasDst_ = true;
    return *this;
}

TInstruction& TInstruction::SetData(TWord data) {
    Data_ = data;
    return *this;
}

std::optional<TError> TInstruction::Execute(NEmulator::TContext ctx) {

#define SAFE_CALL(arg)                              \
    if (auto err = arg) { return std::move(err); }

#define SAFE_DECLARE(name, init)                    \
    const auto name = init;                         \
    if (!name) { return name.error(); }

    const auto pushStack = [&](auto value) -> std::optional<TError> {
        auto& sp = ctx.Registers.GetStackPointer();
        auto& m = ctx.Memory;

        // reserve memory on the stack and save the value
        if constexpr (std::is_same_v<decltype(value), TLong>) {
            sp -= 4;
            SAFE_CALL(ctx.Memory.WriteLong(sp, value));
        } else {
            sp -= 2;
            SAFE_CALL(ctx.Memory.WriteWord(sp, value));
        }
        return std::nullopt;
    };

    const auto popStack = [&](auto& value) -> std::optional<TError> {
        auto& sp = ctx.Registers.GetStackPointer();
        auto& m = ctx.Memory;

        // dump the value and free memory on the stack
        if constexpr (std::is_same_v<decltype(value), TLong&>) {
            SAFE_DECLARE(val, ctx.Memory.ReadLong(sp));
            value = *val;
            sp += 4;
        } else {
            SAFE_DECLARE(val, ctx.Memory.ReadWord(sp));
            value = *val;
            sp += 2;
        }
        return std::nullopt;
    };

    const auto displaceProgramCounter = [&]() {
        auto& pc = ctx.Registers.PC;
        if (Size_ == Byte) {
            TSignedByte offset = Data_;
            pc += offset;
        } else {
            TSignedWord offset = Data_;
            pc += offset;

            // ignore the parsed word
            if (offset < 0) {
                pc -= 2;
            }
        }
    };

    std::size_t incCount = 1;

    const auto tryIncAddress = [&](TTarget& target, bool hasFlag, bool& usedFlag) {
        if (hasFlag && !usedFlag) {
            target.TryIncrementAddress(ctx, incCount);
        }
        usedFlag = true;
    };

    bool usedSrcInc = false;
    const auto tryIncAddressSrc = [&]() { return tryIncAddress(Src_, HasSrc_, usedSrcInc); };

    bool usedDstInc = false;
    const auto tryIncAddressDst = [&]() { return tryIncAddress(Dst_, HasDst_, usedDstInc); };

    if (HasSrc_) {
        Src_.SetIncOrDecCount(1);
    }
    if (HasDst_) {
        Dst_.SetIncOrDecCount(1);
    }

    switch (Kind_) {
        case AbcdKind: {
            SAFE_DECLARE(srcVal, Src_.ReadByte(ctx));
            SAFE_DECLARE(dstVal, Dst_.ReadByte(ctx));
            const TByte extendFlag = ctx.Registers.GetExtendFlag();

            const TWord binaryResult = *srcVal + *dstVal + extendFlag;

            bool carry = false;
            int lval = (*srcVal & 0x0F) + (*dstVal & 0x0F) + extendFlag;
            if (lval > 9) {
                carry = true;
                lval -= 10;
            }

            int hval = ((*srcVal >> 4) & 0x0F) + ((*dstVal >> 4) & 0x0F) + (carry ? 1 : 0);
            carry = false;

            if (lval >= 16) {
                lval -= 16;
                ++hval;
            }

            if (hval > 9) {
                carry = true;
                hval -= 10;
            }

            const TWord result = ((hval << 4) + lval) & 0xFF;

            SAFE_CALL(Dst_.WriteByte(ctx, result));
            ctx.Registers.SetNegativeFlag(GetMsb(result, Byte));
            ctx.Registers.SetCarryFlag(carry);
            ctx.Registers.SetExtendFlag(carry);
            ctx.Registers.SetOverflowFlag((~binaryResult & result & 0x80) != 0);
            if (result != 0) {
                ctx.Registers.SetZeroFlag(result == 0);
            }
            break;
        }
        case AddKind:
        case AddiKind:
        case AndKind:
        case AndiKind:
        case CmpKind:
        case CmpiKind:
        case CmpmKind:
        case EorKind:
        case EoriKind:
        case OrKind:
        case OriKind:
        case SubKind:
        case SubiKind: {
            SAFE_DECLARE(srcVal, Src_.ReadAsLongLong(ctx, Size_));
            tryIncAddressSrc();
            SAFE_DECLARE(dstVal, Dst_.ReadAsLongLong(ctx, Size_));

            const auto type = GetOpcodeType(Kind_);
            const TLongLong result = DoBinaryOp(type, *srcVal, *dstVal);
            if (type != CmpType) {
                SAFE_CALL(Dst_.WriteSized(ctx, result, Size_));
            }

            const bool carry = IsCarry(result, Size_);
            if (type == AddType || type == SubType) {
                ctx.Registers.SetExtendFlag(carry);
            }
            ctx.Registers.SetNegativeFlag(GetMsb(result, Size_));
            ctx.Registers.SetZeroFlag(IsZero(result, Size_));
            if (type == AddType || type == SubType || type == CmpType) {
                ctx.Registers.SetOverflowFlag(IsOverflow(*srcVal, *dstVal, result, Size_, type));
                ctx.Registers.SetCarryFlag(carry);
            } else {
                ctx.Registers.SetOverflowFlag(0);
                ctx.Registers.SetCarryFlag(0);
            }
            break;
        }
        case AddaKind:
        case CmpaKind:
        case SubaKind: {
            const auto type = GetOpcodeType(Kind_);

            TLongLong src;
            if (Size_ == Word) {
                SAFE_DECLARE(srcVal, Src_.ReadWord(ctx));
                src = static_cast<TSignedLongLong>(static_cast<TSignedWord>(*srcVal));
            } else {
                SAFE_DECLARE(srcVal, Src_.ReadLong(ctx));
                src = *srcVal;
            }
            SAFE_DECLARE(dstVal, Dst_.ReadLong(ctx));
            const TLongLong result = DoBinaryOp(type, src, *dstVal);

            if (type == CmpType) {
                const bool carry = IsCarry(result ^ src, Long);
                ctx.Registers.SetNegativeFlag(GetMsb(result, Long));
                ctx.Registers.SetZeroFlag(IsZero(result, Long));
                ctx.Registers.SetOverflowFlag(IsOverflow(src, *dstVal, result, Long, type));
                ctx.Registers.SetCarryFlag(carry);
            } else {
                SAFE_CALL(Dst_.WriteSized(ctx, result, Long));
            }
            break;
        }
        case AddqKind:
        case SubqKind: {
            const auto type = GetOpcodeType(Kind_);
            const TLongLong srcVal = Data_ ? Data_ : 8;
            SAFE_DECLARE(dstVal, Dst_.ReadAsLongLong(ctx, Size_));
            const TLongLong result = DoBinaryOp(type, srcVal, *dstVal);
            SAFE_CALL(Dst_.WriteSized(ctx, result, Size_));

            if (Dst_.GetKind() != TTarget::AddressRegisterKind) {
                const bool carry = IsCarry(result, Size_);
                ctx.Registers.SetNegativeFlag(GetMsb(result, Size_));
                ctx.Registers.SetCarryFlag(carry);
                ctx.Registers.SetExtendFlag(carry);
                ctx.Registers.SetOverflowFlag(IsOverflow(srcVal, *dstVal, result, Size_, type));
                ctx.Registers.SetZeroFlag(IsZero(result, Size_));
            }
            break;
        }
        case AddxKind:
        case SubxKind: {
            const auto type = GetOpcodeType(Kind_);
            SAFE_DECLARE(srcVal, Src_.ReadAsLongLong(ctx, Size_));
            SAFE_DECLARE(dstVal, Dst_.ReadAsLongLong(ctx, Size_));
            const TLongLong result = DoBinaryOp(type, *srcVal + ctx.Registers.GetExtendFlag(), *dstVal);
            SAFE_CALL(Dst_.WriteSized(ctx, result, Size_));

            const bool carry = IsCarry(result, Size_);
            ctx.Registers.SetNegativeFlag(GetMsb(result, Size_));
            ctx.Registers.SetCarryFlag(carry);
            ctx.Registers.SetExtendFlag(carry);
            ctx.Registers.SetOverflowFlag(IsOverflow(*srcVal, *dstVal, result, Size_, type));
            if (!IsZero(result, Size_)) {
                ctx.Registers.SetZeroFlag(0);
            }
            break;
        }
        case AndiToCcrKind:
        case EoriToCcrKind:
        case OriToCcrKind: {
            SAFE_DECLARE(srcVal, Src_.ReadByte(ctx));
            auto& sr = ctx.Registers.SR;
            sr = (sr & ~0xFF) | DoBinaryOp(GetOpcodeType(Kind_), sr & 0xFF, *srcVal);
            break;
        }
        case MoveToCcrKind: {
            SAFE_DECLARE(srcVal, Src_.ReadWord(ctx));
            auto& sr = ctx.Registers.SR;
            sr = (sr & ~0xFF) | (*srcVal & 0xFF);
            break;
        }
        case AndiToSrKind:
        case EoriToSrKind:
        case OriToSrKind: {
            SAFE_DECLARE(srcVal, Src_.ReadWord(ctx));
            // TODO: find out why bits 12 and 14 matter
            ctx.Registers.SR = DoBinaryOp(GetOpcodeType(Kind_), ctx.Registers.SR, *srcVal & 0b1010'1111'1111'1111);
            break;
        }
        case MoveToSrKind: {
            SAFE_DECLARE(srcVal, Src_.ReadWord(ctx));
            tryIncAddressSrc();
            // TODO: find out why bits 12 and 14 matter
            ctx.Registers.SR = *srcVal & 0b1010'1111'1111'1111;
            break;
        }
        case MoveFromSrKind: {
            SAFE_CALL(Dst_.WriteWord(ctx, ctx.Registers.SR));
            break;
        }
        case MoveToUspKind: {
            SAFE_DECLARE(srcVal, Src_.ReadLong(ctx));
            ctx.Registers.USP = *srcVal;
            break;
        }
        case MoveFromUspKind: {
            SAFE_CALL(Dst_.WriteLong(ctx, ctx.Registers.USP));
            break;
        }
        case AslKind:
        case AsrKind:
        case LslKind:
        case LsrKind:
        case RolKind:
        case RorKind:
        case RoxlKind:
        case RoxrKind: {
            const bool isArithmetic = Kind_ == AslKind || Kind_ == AsrKind;
            const bool isRotate = Kind_ == RolKind || Kind_ == RorKind;
            const bool isExtendRotate = Kind_ == RoxlKind || Kind_ == RoxrKind;
            const bool isLeft = Kind_ == AslKind || Kind_ == LslKind || Kind_ == RolKind || Kind_ == RoxlKind;

            SAFE_DECLARE(dstVal, Dst_.ReadAsLongLong(ctx, Size_));

            uint8_t rotation;
            if (HasSrc_) {
                SAFE_DECLARE(srcVal, Src_.ReadAsLongLong(ctx, Size_));
                rotation = *srcVal % 64;
            } else {
                rotation = Data_ ? Data_ : 8;
            }

            TLongLong result = *dstVal;
            bool hasOverflow = false;
            bool curMsb = GetMsb(result, Size_);
            bool lastBitShifted;
            for (int i = 0; i < rotation; ++i) {
                if (isLeft) {
                    lastBitShifted = GetMsb(result, Size_);
                    result <<= 1;
                    if (isRotate) {
                        result |= lastBitShifted;
                    } else if (isExtendRotate) {
                        result |= ctx.Registers.GetExtendFlag();
                        ctx.Registers.SetExtendFlag(lastBitShifted);
                        ctx.Registers.SetCarryFlag(lastBitShifted);
                    }
                } else {
                    if (i >= BitCount(Size_) && isArithmetic) {
                        lastBitShifted = 0;
                    } else {
                        lastBitShifted = result & 1;
                    }
                    if (isArithmetic) {
                        // preserve the most significant bit
                        result = (result >> 1) | (result & (1LL << (BitCount(Size_) - 1)));
                    } else {
                        result >>= 1;
                        if (isRotate) {
                            result |= static_cast<TLongLong>(lastBitShifted) << (BitCount(Size_) - 1);
                        }
                        if (isExtendRotate) {
                            result |= static_cast<TLongLong>(ctx.Registers.GetExtendFlag()) << (BitCount(Size_) - 1);
                            ctx.Registers.SetExtendFlag(lastBitShifted);
                        }
                    }
                }
                bool newMsb = GetMsb(result, Size_);
                if (curMsb != newMsb) {
                    hasOverflow = true;
                }
                curMsb = newMsb;
            }

            SAFE_CALL(Dst_.WriteSized(ctx, result, Size_));

            ctx.Registers.SetNegativeFlag(GetMsb(result, Size_));
            ctx.Registers.SetZeroFlag(IsZero(result, Size_));
            if (isArithmetic) {
                ctx.Registers.SetOverflowFlag(hasOverflow);
            } else {
                ctx.Registers.SetOverflowFlag(0);
            }
            if (rotation == 0) {
                ctx.Registers.SetCarryFlag(0);
                if (isExtendRotate) {
                    ctx.Registers.SetCarryFlag(ctx.Registers.GetExtendFlag());
                }
            } else {
                if (!isRotate && !isExtendRotate) {
                    ctx.Registers.SetExtendFlag(lastBitShifted);
                }
                ctx.Registers.SetCarryFlag(lastBitShifted);
            }
            break;
        }
        case BccKind: {
            if (CalculateCondition(ctx.Registers, Cond_)) {
                displaceProgramCounter();
                if (ctx.Registers.PC & 1) {
                    return TError{TError::UnalignedProgramCounter, "program counter set at %#04x", ctx.Registers.PC};
                }
            }
            break;
        }
        case DbccKind: {
            if (!CalculateCondition(ctx.Registers, Cond_)) {
                SAFE_DECLARE(dstVal, Dst_.ReadWord(ctx));
                TSignedWord counter = *dstVal;
                --counter;
                SAFE_CALL(Dst_.WriteWord(ctx, counter));
                if (counter != -1) {
                    // Dirty hack because BDcc has special displacement
                    if (static_cast<TSignedWord>(Data_) >= 0) {
                        ctx.Registers.PC -= 2;
                    }
                    displaceProgramCounter();
                    if (ctx.Registers.PC & 1) {
                        return TError{TError::UnalignedProgramCounter, "program counter set at %#04x", ctx.Registers.PC};
                    }
                }
            }
            break;
        }
        case SccKind: {
            if (CalculateCondition(ctx.Registers, Cond_)) {
                SAFE_CALL(Dst_.WriteByte(ctx, 0xFF));
            } else {
                SAFE_CALL(Dst_.WriteByte(ctx, 0x00));
            }
            break;
        }
        case BsrKind: {
            pushStack(ctx.Registers.PC);
            displaceProgramCounter();
            if (ctx.Registers.PC & 1) {
                return TError{TError::UnalignedProgramCounter, "program counter set at %#04x", ctx.Registers.PC};
            }
            break;
        }
        case JmpKind:
        case JsrKind: {
            TLong oldPc = ctx.Registers.PC;
            ctx.Registers.PC = Dst_.GetEffectiveAddress(ctx);
            if (Kind_ == JsrKind) {
                pushStack(oldPc);
            }
            if (ctx.Registers.PC & 1) {
                return TError{TError::UnalignedProgramCounter, "program counter set at %#04x", ctx.Registers.PC};
            }
            break;
        }
        case LeaKind: {
            SAFE_CALL(Dst_.WriteLong(ctx, Src_.GetEffectiveAddress(ctx)));
            break;
        }
        case PeaKind: {
            pushStack(Src_.GetEffectiveAddress(ctx));
            break;
        }
        case BchgKind:
        case BclrKind:
        case BsetKind:
        case BtstKind: {
            // read bit number
            SAFE_DECLARE(srcVal, Src_.ReadByte(ctx));
            auto bitNum = *srcVal;
            if (Dst_.GetKind() == TTarget::DataRegisterKind) {
                bitNum %= 32;
            } else {
                bitNum %= 8;
            }

            // read destination value
            TLongLong val;
            if (Dst_.GetKind() == TTarget::DataRegisterKind) {
                SAFE_DECLARE(dstVal, Dst_.ReadLong(ctx));
                val = *dstVal;
            } else {
                SAFE_DECLARE(dstVal, Dst_.ReadByte(ctx));
                val = *dstVal;
            }

            const auto mask = 1LL << bitNum;
            auto newVal = val;
            if (Kind_ == BchgKind) {
                newVal ^= mask;
            } else if (Kind_ == BclrKind) {
                newVal &= newVal ^ mask;
            } else if (Kind_ == BsetKind) {
                newVal |= mask;
            }

            // update Z flag and write value
            ctx.Registers.SetZeroFlag(not (val & mask));
            if (newVal != val) {
                if (Dst_.GetKind() == TTarget::DataRegisterKind) {
                    SAFE_CALL(Dst_.WriteLong(ctx, newVal));
                } else {
                    SAFE_CALL(Dst_.WriteByte(ctx, newVal));
                }
            }

            break;
        }
        case ClrKind:
        case NegKind:
        case NegxKind:
        case NotKind: {
            SAFE_DECLARE(dstVal, Dst_.ReadAsLongLong(ctx, Size_));
            auto result = *dstVal;

            bool hasOverflow = false;

            if (Kind_ == ClrKind) {
                result = 0;
            } else if (Kind_ == NotKind) {
                result = ~result;
            } else if (Kind_ == NegKind || Kind_ == NegxKind) {
                result = ~result;

                if (Kind_ != NegxKind || !ctx.Registers.GetExtendFlag()) {
                    const auto mask0 = (1LL << (BitCount(Size_) - 1)) - 1;
                    const auto mask1 = (1LL << BitCount(Size_)) - 1;
                    if ((result & mask1) == mask0) {
                        hasOverflow = true;
                    }
                    ++result;
                }
            }

            SAFE_CALL(Dst_.WriteSized(ctx, result, Size_));

            ctx.Registers.SetNegativeFlag(GetMsb(result, Size_));
            const bool isZero = IsZero(result, Size_);
            if (Kind_ != NegxKind || !isZero) {
                ctx.Registers.SetZeroFlag(isZero);
            }
            if (Kind_ == NegKind || Kind_ == NegxKind) {
                ctx.Registers.SetOverflowFlag(hasOverflow);
                ctx.Registers.SetCarryFlag(IsCarry(result, Size_));
                ctx.Registers.SetExtendFlag(ctx.Registers.GetCarryFlag());
            } else {
                ctx.Registers.SetOverflowFlag(0);
                ctx.Registers.SetCarryFlag(0);
            }
            break;
        }
        case MoveKind: {
            auto tmp = ctx.Registers.PC;
            ctx.Registers.PC = Data_;
            SAFE_DECLARE(srcVal, Src_.ReadAsLongLong(ctx, Size_));
            tryIncAddressSrc();
            ctx.Registers.PC = tmp;

            SAFE_CALL(Dst_.WriteSized(ctx, *srcVal, Size_));

            ctx.Registers.SetNegativeFlag(GetMsb(*srcVal, Size_));
            ctx.Registers.SetZeroFlag(IsZero(*srcVal, Size_));
            ctx.Registers.SetOverflowFlag(0);
            ctx.Registers.SetCarryFlag(0);
            break;
        }
        case MoveaKind: {
            auto tmp = ctx.Registers.PC;
            ctx.Registers.PC = Data_;

            TLongLong src;
            if (Size_ == Word) {
                SAFE_DECLARE(srcVal, Src_.ReadWord(ctx));
                src = static_cast<TSignedLongLong>(static_cast<TSignedWord>(*srcVal));
            } else {
                SAFE_DECLARE(srcVal, Src_.ReadLong(ctx));
                src = *srcVal;
            }

            tryIncAddressSrc();
            ctx.Registers.PC = tmp;

            SAFE_CALL(Dst_.WriteLong(ctx, src));
            break;
        }
        case MovemKind: {
            const auto hasBit = [&](std::size_t i) {
                return Data_ & (1 << i);
            };

            const auto getReg = [&](std::size_t i) -> TLong& {
                if (i <= 7) {
                    return ctx.Registers.D[i];
                } else if (i <= 14) {
                    return ctx.Registers.A[i - 8];
                } else {
                    return ctx.Registers.GetStackPointer();
                }
            };

            if (HasSrc_) {
                std::size_t regCount = __builtin_popcount(Data_);
                incCount = regCount;
                SAFE_DECLARE(dataOrErr, Src_.Read(ctx, regCount * Size_));
                auto& data = *dataOrErr;

                std::size_t curPos = 0;
                for (std::size_t i = 0; i < 16; ++i) {
                    if (hasBit(i)) {
                        // a corner case: don't write to the postincrement register
                        if (i < 8 || Src_.GetKind() != TTarget::AddressIncrementKind || i - 8 != Src_.GetIndex()) {
                            if (Size_ == Word) {
                                TSignedLong l = static_cast<TSignedWord>((data[curPos] << 8) + data[curPos + 1]);
                                getReg(i) = l;
                            } else {
                                getReg(i) = (data[curPos] << 24) + (data[curPos + 1] << 16) + (data[curPos + 2] << 8) + data[curPos + 3];
                            }
                        }
                        curPos += Size_;
                    }
                }
            } else {
                TDataHolder data;
                for (std::size_t i = 0; i < 16; ++i) {
                    bool has = hasBit(i);
                    if (Dst_.GetKind() == TTarget::AddressDecrementKind) {
                        has = hasBit(15 - i);
                    }

                    if (has) {
                        const TLong reg = getReg(i);
                        if (Size_ == Long) {
                            data.push_back((reg >> 24) & 0xFF);
                            data.push_back((reg >> 16) & 0xFF);
                        }
                        data.push_back((reg >> 8) & 0xFF);
                        data.push_back(reg & 0xFF);
                    }
                }
                Dst_.SetIncOrDecCount(__builtin_popcount(Data_));
                SAFE_CALL(Dst_.Write(ctx, data));
            }
            break;
        }
        case MoveqKind: {
            TLongLong src = static_cast<TSignedLongLong>(static_cast<TSignedByte>(Data_));
            SAFE_CALL(Dst_.WriteLong(ctx, src));

            ctx.Registers.SetNegativeFlag(GetMsb(src, Long));
            ctx.Registers.SetZeroFlag(IsZero(src, Long));
            ctx.Registers.SetOverflowFlag(0);
            ctx.Registers.SetCarryFlag(0);
            break;
        }
        case SwapKind: {
            SAFE_DECLARE(dstVal, Dst_.ReadLong(ctx));
            TLong val = *dstVal;
            TLong half = val & 0xFFFF;
            val = (val >> 16) | (half << 16);
            SAFE_CALL(Dst_.WriteLong(ctx, val));

            ctx.Registers.SetNegativeFlag(GetMsb(val, Long));
            ctx.Registers.SetZeroFlag(IsZero(val, Long));
            ctx.Registers.SetOverflowFlag(0);
            ctx.Registers.SetCarryFlag(0);
            break;
        }
        case TasKind: {
            SAFE_DECLARE(dstVal, Dst_.ReadByte(ctx));
            TByte newVal = *dstVal | (1 << 7);
            SAFE_CALL(Dst_.WriteByte(ctx, newVal));

            ctx.Registers.SetNegativeFlag(GetMsb(*dstVal, Byte));
            ctx.Registers.SetZeroFlag(IsZero(*dstVal, Byte));
            ctx.Registers.SetOverflowFlag(0);
            ctx.Registers.SetCarryFlag(0);
            break;
        }
        case ExgKind: {
            SAFE_DECLARE(srcVal, Src_.ReadLong(ctx));
            SAFE_DECLARE(dstVal, Dst_.ReadLong(ctx));
            SAFE_CALL(Dst_.WriteLong(ctx, *srcVal));
            SAFE_CALL(Src_.WriteLong(ctx, *dstVal));
            break;
        }
        case ExtKind: {
            TLong val;
            if (Size_ == Word) {
                SAFE_DECLARE(dstVal, Dst_.ReadWord(ctx));
                const TWord ext = *dstVal & (1 << 7) ? 0xFF : 0x00;
                val = (ext << 8) | (*dstVal & 0xFF);
                SAFE_CALL(Dst_.WriteWord(ctx, val));
            } else /* Size_ == Long */ {
                SAFE_DECLARE(dstVal, Dst_.ReadLong(ctx));
                const TLong ext = *dstVal & (1 << 15) ? 0xFFFF : 0x0000;
                val = (ext << 16) | (*dstVal & 0xFFFF);
                SAFE_CALL(Dst_.WriteLong(ctx, val));
            }
            ctx.Registers.SetNegativeFlag(GetMsb(val, Size_));
            ctx.Registers.SetZeroFlag(IsZero(val, Size_));
            ctx.Registers.SetOverflowFlag(0);
            ctx.Registers.SetCarryFlag(0);
            break;
        }
        case LinkKind: {
            SAFE_DECLARE(dstVal, Dst_.ReadLong(ctx));
            auto& sp = ctx.Registers.GetStackPointer();
            if (Dst_.GetIndex() == 7) {
                // a really dirty hack to deal with "[LINK A7, #]"
                pushStack(*dstVal - 4);
            } else {
                pushStack(*dstVal);
            }

            SAFE_CALL(Dst_.WriteLong(ctx, sp));
            TSignedWord offset = Data_;
            sp += offset;
            break;
        }
        case UnlinkKind: {
            SAFE_DECLARE(dstVal, Dst_.ReadLong(ctx));
            ctx.Registers.GetStackPointer() = *dstVal;
            TLong value;
            SAFE_CALL(popStack(value));
            SAFE_CALL(Dst_.WriteLong(ctx, value));
            break;
        }
        case TrapKind:
        case TrapvKind: {
            if (Kind_ == TrapvKind && !ctx.Registers.GetOverflowFlag()) {
                break;
            }

            ctx.Registers.SetSupervisorFlag(1);
            pushStack(ctx.Registers.PC);
            pushStack(ctx.Registers.SR);

            SAFE_DECLARE(newPc, ctx.Memory.ReadLong(Data_ * 4));
            ctx.Registers.PC = *newPc;
            break;
        }
        case RteKind:
        case RtrKind:
        case RtsKind: {
            TWord newSr;
            if (Kind_ != RtsKind) {
                popStack(newSr);
            }
            popStack(ctx.Registers.PC);

            if (Kind_ == RteKind) {
                // TODO: find out why bits 12 and 14 matter
                ctx.Registers.SR = newSr & 0b1010'1111'1111'1111;
            } else if (Kind_ == RtrKind) {
                ctx.Registers.SR = (ctx.Registers.SR & 0xFF00) | (newSr & 0x00FF);
            }

            if (ctx.Registers.PC & 1) {
                return TError{TError::UnalignedProgramCounter, "program counter set at %#04x", ctx.Registers.PC};
            }
            break;
        }
        case TstKind: {
            SAFE_DECLARE(srcVal, Src_.ReadAsLongLong(ctx, Size_));
            ctx.Registers.SetNegativeFlag(GetMsb(*srcVal, Size_));
            ctx.Registers.SetZeroFlag(IsZero(*srcVal, Size_));
            ctx.Registers.SetOverflowFlag(0);
            ctx.Registers.SetCarryFlag(0);
            break;
        }
        case NopKind:
        case ResetKind: {
            break;
        }
    }

    tryIncAddressSrc();
    tryIncAddressDst();

    return std::nullopt;
}

tl::expected<TInstruction, TError> TInstruction::Decode(NEmulator::TContext ctx) {
    const auto readWord = [&ctx]() {
        auto& pc = ctx.Registers.PC;
        const auto word = ctx.Memory.ReadWord(pc);
        if (word) {
            pc += 2;
        }
        return word;
    };

#define READ_WORD_SAFE                       \
    const auto word = readWord();            \
    if (!word) {                             \
        return tl::unexpected(word.error()); \
    }

#define READ_TWO_WORDS_SAFE                                 \
    const auto word0 = readWord();                          \
    if (!word0) { return tl::unexpected(word0.error()); }   \
    const auto word1 = readWord();                          \
    if (!word1) { return tl::unexpected(word0.error()); }

    // read the current word (16 bits)
    READ_WORD_SAFE;

    // helper functions
    const auto applyMask = [&word](TWord mask) { return *word & mask; };
    const auto getBits = [&word](std::size_t begin, std::size_t len) { return (*word >> begin) & ((1 << len) - 1); };
    const auto getBit = [&getBits](std::size_t bit) { return getBits(bit, 1); };

    // 00 -> byte, 01 -> word, 02 -> long
    const auto getSize0 = [&getBits]() {
        switch (getBits(6, 2)) {
            case 0: return Byte;
            case 1: return Word;
            case 2: return Long;
            default: __builtin_unreachable();
        }
    };

    const auto parseTargetWithSize = [&](ESize size, std::size_t modeBegin, std::size_t indexBegin) -> tl::expected<TTarget, TError> {
        TTarget target;

        const auto mode = getBits(modeBegin, 3);
        const auto xn = getBits(indexBegin, 3);

        switch (mode) {
            case 0: {
                target.SetKind(TTarget::DataRegisterKind).SetIndex(xn);
                break;
            }
            case 1: {
                target.SetKind(TTarget::AddressRegisterKind).SetIndex(xn);
                break;
            }
            case 2: {
                target.SetKind(TTarget::AddressKind).SetIndex(xn);
                break;
            }
            case 3: {
                target.SetKind(TTarget::AddressIncrementKind).SetIndex(xn).SetSize(size);
                break;
            }
            case 4: {
                target.SetKind(TTarget::AddressDecrementKind).SetIndex(xn).SetSize(size);
                break;
            }
            case 5: {
                READ_WORD_SAFE;
                target.SetKind(TTarget::AddressDisplacementKind).SetIndex(xn).SetExtWord0(*word);
                break;
            }
            case 6: {
                READ_WORD_SAFE;
                target.SetKind(TTarget::AddressIndexKind).SetIndex(xn).SetExtWord0(*word);
                break;
            }
            case 7: {
                switch (xn) {
                    case 0: {
                        READ_WORD_SAFE;
                        target.SetKind(TTarget::AbsoluteShortKind).SetExtWord0(*word);
                        break;
                    }
                    case 1: {
                        READ_TWO_WORDS_SAFE;
                        target.SetKind(TTarget::AbsoluteLongKind).SetExtWord0(*word0).SetExtWord1(*word1);
                        break;
                    }
                    case 2: {
                        READ_WORD_SAFE;
                        target.SetKind(TTarget::ProgramCounterDisplacementKind).SetExtWord0(*word);
                        break;
                    }
                    case 3: {
                        READ_WORD_SAFE;
                        target.SetKind(TTarget::ProgramCounterIndexKind).SetExtWord0(*word);
                        break;
                    }
                    case 4: {
                        auto& pc = ctx.Registers.PC;
                        target.SetKind(TTarget::ImmediateKind).SetAddress((size == Byte) ? (pc + 1) : pc);
                        pc += (size == Long) ? 4 : 2;
                        break;
                    }
                    default: {
                        return tl::unexpected<TError>(TError::UnknownAddressingMode, "Unknown addresing mode in word %#04x", *word);
                    }
                }
                break;
            }
            default: {
                __builtin_unreachable();
            }
        }

        return target;
    };

#define PARSE_TARGET_WITH_SIZE_SAFE(size)               \
    auto dst = parseTargetWithSize(size, 3, 0);         \
    if (!dst) { return tl::unexpected(dst.error()); }

#define PARSE_TARGET_WITH_ARGS_SAFE(dst, size, modeBegin, indexBegin)  \
    auto dst = parseTargetWithSize(size, modeBegin, indexBegin);       \
    if (!dst) { return tl::unexpected(dst.error()); }

#define PARSE_TARGET_SAFE PARSE_TARGET_WITH_SIZE_SAFE(getSize0())

    // decode the opcode
    TInstruction inst;

    /*
     * Status register opcodes: [ANDI|EORI]to[CCR|SR]
     */
    const auto tryParseStatusRegisterOpcodes = [&]() -> tl::expected<bool, TError> {
        using TCase = std::tuple<EKind, EKind, int>;
        constexpr std::array<TCase, 3> cases{
            std::make_tuple(OriToCcrKind, OriToSrKind, 0),
            std::make_tuple(AndiToCcrKind, AndiToSrKind, 1),
            std::make_tuple(EoriToCcrKind, EoriToSrKind, 5),
        };
        for (auto [ccrKind, srKind, index] : cases) {
            if (applyMask(0b1111'0001'1011'1111) == 0b0000'0000'0011'1100 && getBits(9, 3) == index) {
                bool isWord = getBit(6);

                auto& pc = ctx.Registers.PC;
                auto src = TTarget{}.SetKind(TTarget::ImmediateKind).SetAddress(pc + (isWord ? 0 : 1));
                pc += 2;

                inst.SetKind(isWord ? srKind : ccrKind).SetSrc(src);
                return true;
            }
        }
        return false;
    };

    /*
     * Bit manipulation opcodes: BTST, BCHG, BCLR, BSET
     */
    const auto tryParseBitOpcodes = [&]() -> tl::expected<bool, TError> {
        using TCase = std::tuple<EKind, TWord, TWord>;
        constexpr std::array<TCase, 4> cases{
            std::make_tuple(BtstKind, 0b0000'0001'0000'0000, 0b0000'1000'0000'0000),
            std::make_tuple(BchgKind, 0b0000'0001'0100'0000, 0b0000'1000'0100'0000),
            std::make_tuple(BclrKind, 0b0000'0001'1000'0000, 0b0000'1000'1000'0000),
            std::make_tuple(BsetKind, 0b0000'0001'1100'0000, 0b0000'1000'1100'0000),
        };
        for (auto [kind, registerMask, immediateMask] : cases) {
            if (applyMask(0b1111'0001'1100'0000) == registerMask) {
                auto src = TTarget{}.SetKind(TTarget::DataRegisterKind).SetIndex(getBits(9, 3));
                PARSE_TARGET_WITH_SIZE_SAFE(Byte);
                inst.SetKind(kind).SetSrc(src).SetDst(*dst).SetSize(Byte);
                return true;
            }
            if (applyMask(0b1111'1111'1100'0000) == immediateMask) {
                auto& pc = ctx.Registers.PC;
                auto src = TTarget{}.SetKind(TTarget::ImmediateKind).SetAddress(pc + 1);
                pc += 2;

                PARSE_TARGET_WITH_SIZE_SAFE(Byte);
                inst.SetKind(kind).SetSrc(src).SetDst(*dst).SetSize(Byte);
                return true;
            }
        }
        return false;
    };

    /*
     * Unary operations: NEG, NEGX, CLR, NOT
     */
    const auto tryParseUnaryOpcodes = [&]() -> tl::expected<bool, TError> {
        using TCase = std::tuple<EKind, TWord>;
        constexpr std::array<TCase, 4> cases{
            std::make_tuple(NegxKind, 0b0100'0000'0000'0000),
            std::make_tuple(ClrKind,  0b0100'0010'0000'0000),
            std::make_tuple(NegKind,  0b0100'0100'0000'0000),
            std::make_tuple(NotKind,  0b0100'0110'0000'0000),
        };
        for (auto [kind, mask] : cases) {
            if (applyMask(0b1111'1111'0000'0000) == mask && getBits(6, 2) != 3) {
                PARSE_TARGET_SAFE;
                inst.SetKind(kind).SetDst(*dst).SetSize(getSize0());
                return true;
            }
        }
        return false;
    };

    /*
     * Bit shift operations: ASL, ASR, LSL, LSR, ROL, ROR, ROXL, ROXR
     */
    const auto tryParseShiftOpcodes = [&]() -> tl::expected<bool, TError> {
        using TCase = std::tuple<EKind, EKind, int>;
        constexpr std::array<TCase, 4> cases{
            std::make_tuple(AslKind, AsrKind, 0),
            std::make_tuple(LslKind, LsrKind, 1),
            std::make_tuple(RoxlKind, RoxrKind, 2),
            std::make_tuple(RolKind, RorKind, 3),
        };

        for (auto [leftKind, rightKind, index] : cases) {
            if (applyMask(0b1111'1000'1100'0000) == 0b1110'0000'1100'0000 && getBits(9, 2) == index) {
                // operation on any memory, shift by 1
                auto kind = getBit(8) ? leftKind : rightKind;
                PARSE_TARGET_WITH_SIZE_SAFE(Word);
                inst.SetKind(kind).SetDst(*dst).SetSize(Word).SetData(1);
                return true;
            }
            if (applyMask(0b1111'0000'0000'0000) == 0b1110'0000'0000'0000 && getBits(3, 2) == index && getBits(6, 2) != 3) {
                // operation on Dn
                auto kind = getBit(8) ? leftKind : rightKind;
                uint8_t rotation = getBits(9, 3);
                auto dst = TTarget{}.SetKind(TTarget::DataRegisterKind).SetIndex(getBits(0, 3));

                inst.SetKind(kind).SetDst(dst).SetSize(getSize0());
                if (getBit(5)) {
                    // shift count is in the data register
                    auto src = TTarget{}.SetKind(TTarget::DataRegisterKind).SetIndex(rotation);
                    inst.SetSrc(src);
                } else {
                    // shift count is immediate
                    inst.SetData(rotation);
                }
                return true;
            }
        }
        return false;
    };

    /*
     * Binary operations on immediate: ADDI, ANDI, EORI, ORI
     */
    const auto tryParseBinaryOnImmediateOpcodes = [&]() -> tl::expected<bool, TError> {
        using TCase = std::tuple<EKind, int>;
        constexpr std::array<TCase, 6> cases{
            std::make_tuple(OriKind, 0),
            std::make_tuple(AndiKind, 1),
            std::make_tuple(SubiKind, 2),
            std::make_tuple(AddiKind, 3),
            std::make_tuple(EoriKind, 5),
            std::make_tuple(CmpiKind, 6),
        };

        for (auto [kind, index] : cases) {
            if (applyMask(0b1111'0001'0000'0000) == 0b0000'0000'0000'0000 && getBits(9, 3) == index) {
                auto& pc = ctx.Registers.PC;
                auto src = TTarget{}.SetKind(TTarget::ImmediateKind).SetAddress((getSize0() == Byte) ? (pc + 1) : pc);
                pc += (getSize0() == Long) ? 4 : 2;

                PARSE_TARGET_SAFE;
                inst.SetKind(kind).SetSrc(src).SetDst(*dst).SetSize(getSize0());
                return true;
            }
        }
        return false;
    };

    /*
     * Binary operations: ADD, AND, EOR, OR, SUB
     */
    const auto tryParseBinaryOpcodes = [&]() -> tl::expected<bool, TError> {
        using TCase = std::tuple<EKind, int>;
        constexpr std::array<TCase, 5> cases{
            std::make_tuple(OrKind, 0),
            std::make_tuple(SubKind, 1),
            std::make_tuple(EorKind, 3),
            std::make_tuple(AndKind, 4),
            std::make_tuple(AddKind, 5),
        };

        for (auto [kind, index] : cases) {
            if (applyMask(0b1000'0000'0000'0000) == 0b1000'0000'0000'0000 && getBits(12, 3) == index) {
                auto src = TTarget{}.SetKind(TTarget::DataRegisterKind).SetIndex(getBits(9, 3));
                PARSE_TARGET_SAFE;
                if (!getBit(8)) {
                    if (kind == EorKind) {
                        // some hack
                        kind = CmpKind;
                    }
                    std::swap(src, *dst);
                }
                inst.SetKind(kind).SetSrc(src).SetDst(*dst).SetSize(getSize0());
                return true;
            }
        }
        return false;
    };

    /*
     * Binary operations on address: ADDA, SUBA
     */
    const auto tryParseBinaryOnAddressOpcodes = [&]() -> tl::expected<bool, TError> {
        using TCase = std::tuple<EKind, int>;
        constexpr std::array<TCase, 3> cases{
            std::make_tuple(SubaKind, 0),
            std::make_tuple(CmpaKind, 1),
            std::make_tuple(AddaKind, 2),
        };

        for (auto [kind, index] : cases) {
            if (applyMask(0b1001'0000'1100'0000) == 0b1001'0000'1100'0000 && getBits(13, 2) == index) {
                const auto size = getBit(8) ? Long : Word;
                auto src = TTarget{}.SetKind(TTarget::AddressRegisterKind).SetIndex(getBits(9, 3));
                PARSE_TARGET_WITH_SIZE_SAFE(size);
                std::swap(src, *dst);
                inst.SetKind(kind).SetSrc(src).SetDst(*dst).SetSize(size);
                return true;
            }
        }
        return false;
    };

    /*
     * Moves: MOVE, MOVEM, MOVEA, MOVEQ, MOVEtoCCR, MOVE[to|from][SR|USP]
     */
    const auto tryParseMoveOpcodes = [&]() -> tl::expected<bool, TError> {
        // MOVE/MOVEA
        if (applyMask(0b1100'0000'0000'0000) == 0b0000'0000'0000'0000) {
            std::optional<ESize> size;
            switch (getBits(12, 2)) {
                case 0b01: size = Byte; break;
                case 0b11: size = Word; break;
                case 0b10: size = Long; break;
                default: break;
            }
            if (size) {
                PARSE_TARGET_WITH_ARGS_SAFE(src, *size, 3, 0);
                TLong pc = ctx.Registers.PC; // remember current program counter
                PARSE_TARGET_WITH_ARGS_SAFE(dst, *size, 6, 9);
                const auto kind = getBits(6, 3) == 1 ? MoveaKind : MoveKind;
                inst.SetKind(kind).SetSrc(*src).SetDst(*dst).SetSize(*size).SetData(pc);
                return true;
            }
        }
        // MOVEM
        if (applyMask(0b1111'1011'1000'0000) == 0b0100'1000'1000'0000) {
            READ_WORD_SAFE;
            const auto size = getBit(6) ? Long : Word;
            PARSE_TARGET_WITH_SIZE_SAFE(size);
            inst.SetKind(MovemKind).SetData(*word).SetSize(size);
            if (getBit(10)) {
                inst.SetSrc(*dst);
            } else {
                inst.SetDst(*dst);
            }
            return true;
        }
        // MOVEQ
        if (applyMask(0b1111'0001'0000'0000) == 0b0111'0000'0000'0000) {
            auto dst = TTarget{}.SetKind(TTarget::DataRegisterKind).SetIndex(getBits(9, 3));
            inst.SetKind(MoveqKind).SetData(getBits(0, 8)).SetDst(dst);
            return true;
        }
        // MOVEtoCCR/MOVEtoSR
        if (applyMask(0b1111'1101'1100'0000) == 0b0100'0100'1100'0000) {
            PARSE_TARGET_WITH_SIZE_SAFE(Word);
            inst.SetKind(getBit(9) ? MoveToSrKind : MoveToCcrKind).SetSrc(*dst);
            return true;
        }
        // MOVEfromSR
        if (applyMask(0b1111'1111'1100'0000) == 0b0100'0000'1100'0000) {
            PARSE_TARGET_WITH_SIZE_SAFE(Word);
            inst.SetKind(MoveFromSrKind).SetDst(*dst);
            return true;
        }
        // MOVEtoUSP
        if (applyMask(0b1111'1111'1111'1000) == 0b0100'1110'0110'0000) {
            auto src = TTarget{}.SetKind(TTarget::AddressRegisterKind).SetIndex(getBits(0, 3));
            inst.SetKind(MoveToUspKind).SetSrc(src);
            return true;
        }
        // MOVEfromUSP
        if (applyMask(0b1111'1111'1111'1000) == 0b0100'1110'0110'1000) {
            auto dst = TTarget{}.SetKind(TTarget::AddressRegisterKind).SetIndex(getBits(0, 3));
            inst.SetKind(MoveFromUspKind).SetDst(dst);
            return true;
        }
        return false;
    };

    if (*word == 0b0100'1110'0111'0000) {
        inst.SetKind(ResetKind);
    }
    else if (*word == 0b0100'1110'0111'0001) {
        inst.SetKind(NopKind);
    }
    else if (applyMask(0b1111'0000'1111'1000) == 0b0101'0000'1100'1000) {
        const auto cond = static_cast<ECondition>(getBits(8, 4));
        auto dst = TTarget{}.SetKind(TTarget::DataRegisterKind).SetIndex(getBits(0, 3)).SetSize(Word);
        READ_WORD_SAFE;
        inst.SetKind(DbccKind).SetCondition(cond).SetDst(dst).SetData(*word).SetSize(Word);
    }
    else if (applyMask(0b1111'0000'1100'0000) == 0b0101'0000'1100'0000) {
        const auto cond = static_cast<ECondition>(getBits(8, 4));
        PARSE_TARGET_WITH_SIZE_SAFE(Byte);
        inst.SetKind(SccKind).SetCondition(cond).SetDst(*dst);
    }
    else if (applyMask(0b1111'0000'0000'0000) == 0b0101'0000'0000'0000) {
        PARSE_TARGET_SAFE;
        inst.SetKind(getBit(8) ? SubqKind : AddqKind).SetData(getBits(9, 3)).SetDst(*dst).SetSize(getSize0());
    }
    else if (applyMask(0b1111'0001'1111'0000) == 0b1100'0001'0000'0000) {
        const auto kind = getBit(3) ? TTarget::AddressDecrementKind : TTarget::DataRegisterKind;
        auto src = TTarget{}.SetKind(kind).SetIndex(getBits(0, 3)).SetSize(1);
        auto dst = TTarget{}.SetKind(kind).SetIndex(getBits(9, 3)).SetSize(1);
        inst.SetKind(AbcdKind).SetSrc(src).SetDst(dst);
    }
    else if (applyMask(0b1011'0001'0011'0000) == 0b1001'0001'0000'0000 && getBits(6, 2) != 3) {
        const auto size = getSize0();
        const auto kind = getBit(3) ? TTarget::AddressDecrementKind : TTarget::DataRegisterKind;
        auto src = TTarget{}.SetKind(kind).SetIndex(getBits(0, 3)).SetSize(size);
        auto dst = TTarget{}.SetKind(kind).SetIndex(getBits(9, 3)).SetSize(size);
        inst.SetKind(getBit(14) ? AddxKind : SubxKind).SetSrc(src).SetDst(dst).SetSize(size);
    }
    else if (applyMask(0b1111'0000'0000'0000) == 0b0110'0000'0000'0000) {
        const auto cond = static_cast<ECondition>(getBits(8, 4));

        auto displacement = getBits(0, 8);
        auto size = Byte;
        if (displacement == 0) {
            READ_WORD_SAFE;
            displacement = *word;
            size = Word;
        }

        // the False condition is actually a BSR (Branch to Subroutine)
        if (cond == FalseCond) {
            inst.SetKind(BsrKind).SetData(displacement).SetSize(size);
        } else {
            inst.SetKind(BccKind).SetCondition(cond).SetData(displacement).SetSize(size);
        }
    }
    else if (applyMask(0b1111'1111'1000'0000) == 0b0100'1110'1000'0000) {
        PARSE_TARGET_WITH_SIZE_SAFE(Long);
        const auto kind = getBit(6) ? JmpKind : JsrKind;
        inst.SetKind(kind).SetDst(*dst);
    }
    else if (applyMask(0b1111'0001'1100'0000) == 0b0100'0001'1100'0000) {
        PARSE_TARGET_WITH_SIZE_SAFE(Long);
        auto src = TTarget{}.SetKind(TTarget::AddressRegisterKind).SetIndex(getBits(9, 3));
        std::swap(src, *dst);
        inst.SetKind(LeaKind).SetSrc(src).SetDst(*dst);
    }
    else if (applyMask(0b1111'0001'0011'1000) == 0b1011'0001'0000'1000 && getBits(6, 2) != 3) {
        const auto size = getSize0();
        auto src = TTarget{}.SetKind(TTarget::AddressIncrementKind).SetIndex(getBits(0, 3)).SetSize(size);
        auto dst = TTarget{}.SetKind(TTarget::AddressIncrementKind).SetIndex(getBits(9, 3)).SetSize(size);
        inst.SetKind(CmpmKind).SetSrc(src).SetDst(dst).SetSize(size);
    }
    else if (applyMask(0b1111'1111'1111'1000) == 0b0100'1000'0100'0000) {
        auto dst = TTarget{}.SetKind(TTarget::DataRegisterKind).SetIndex(getBits(0, 3));
        inst.SetKind(SwapKind).SetDst(dst);
    }
    else if (applyMask(0b1111'1111'1100'0000) == 0b0100'1000'0100'0000) {
        PARSE_TARGET_WITH_SIZE_SAFE(Long);
        inst.SetKind(PeaKind).SetSrc(*dst);
    }
    else if (applyMask(0b1111'1111'1100'0000) == 0b0100'1010'1100'0000) {
        PARSE_TARGET_WITH_SIZE_SAFE(Byte);
        inst.SetKind(TasKind).SetDst(*dst);
    }
    else if (applyMask(0b1111'0001'0011'0000) == 0b1100'0001'0000'0000) {
        auto src = TTarget{}.SetIndex(getBits(0, 3));
        auto dst = TTarget{}.SetIndex(getBits(9, 3));
        if (getBits(3, 5) == 0b01000) {
            src.SetKind(TTarget::DataRegisterKind);
            dst.SetKind(TTarget::DataRegisterKind);
        }
        else if (getBits(3, 5) == 0b01001) {
            src.SetKind(TTarget::AddressRegisterKind);
            dst.SetKind(TTarget::AddressRegisterKind);
        }
        else {
            src.SetKind(TTarget::AddressRegisterKind);
            dst.SetKind(TTarget::DataRegisterKind);
        }
        inst.SetKind(ExgKind).SetSrc(src).SetDst(dst);
    }
    else if (applyMask(0b1111'1111'1011'1000) == 0b0100'1000'1000'0000) {
        auto dst = TTarget{}.SetKind(TTarget::DataRegisterKind).SetIndex(getBits(0, 3));
        inst.SetKind(ExtKind).SetDst(dst).SetSize(getBit(6) ? Long : Word);
    }
    else if (applyMask(0b1111'1111'1111'1000) == 0b0100'1110'0101'0000) {
        auto dst = TTarget{}.SetKind(TTarget::AddressRegisterKind).SetIndex(getBits(0, 3));
        READ_WORD_SAFE;
        inst.SetKind(LinkKind).SetDst(dst).SetData(*word);
    }
    else if (applyMask(0b1111'1111'1111'1000) == 0b0100'1110'0101'1000) {
        auto dst = TTarget{}.SetKind(TTarget::AddressRegisterKind).SetIndex(getBits(0, 3));
        inst.SetKind(UnlinkKind).SetDst(dst);
    }
    else if (applyMask(0b1111'1111'1111'0000) == 0b0100'1110'0100'0000) {
        constexpr int TRAP_VECTOR_OFFSET = 32;
        inst.SetKind(TrapKind).SetData(TRAP_VECTOR_OFFSET + getBits(0, 4));
    }
    else if (applyMask(0b1111'1111'1111'1111) == 0b0100'1110'0111'0110) {
        inst.SetKind(TrapvKind).SetData(7);
    }
    else if (applyMask(0b1111'1111'1111'1111) == 0b0100'1110'0111'0011) {
        inst.SetKind(RteKind);
    }
    else if (applyMask(0b1111'1111'1111'1111) == 0b0100'1110'0111'0101) {
        inst.SetKind(RtsKind);
    }
    else if (applyMask(0b1111'1111'1111'1111) == 0b0100'1110'0111'0111) {
        inst.SetKind(RtrKind);
    }
    else if (applyMask(0b1111'1111'0000'0000) == 0b0100'1010'0000'0000) {
        PARSE_TARGET_SAFE;
        inst.SetKind(TstKind).SetSrc(*dst).SetSize(getSize0());
    }
    else {

#define TRY_PARSE_SAFE(func)                                    \
        {                                                       \
            auto res = func();                                  \
            if (!res) { return tl::unexpected(res.error()); }   \
            if (*res) { return inst; }                          \
        }

        TRY_PARSE_SAFE(tryParseStatusRegisterOpcodes);
        TRY_PARSE_SAFE(tryParseBitOpcodes);
        TRY_PARSE_SAFE(tryParseUnaryOpcodes);
        TRY_PARSE_SAFE(tryParseShiftOpcodes);
        TRY_PARSE_SAFE(tryParseBinaryOnAddressOpcodes);
        TRY_PARSE_SAFE(tryParseBinaryOnImmediateOpcodes);
        TRY_PARSE_SAFE(tryParseBinaryOpcodes);
        TRY_PARSE_SAFE(tryParseMoveOpcodes);

        return tl::unexpected<TError>(TError::UnknownOpcode, "Unknown opcode %#04x", *word);
    }

    return inst;
}

} // namespace NOpcodes
