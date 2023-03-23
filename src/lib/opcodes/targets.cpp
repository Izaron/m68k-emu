#include "targets.h"

#include <type_traits>

namespace NOpcodes {

static_assert(std::is_trivially_constructible_v<TTarget>);

namespace {

TLong& GetAReg(NRegisters::TRegisters& r, int index) {
    if (index < 7) {
        return r.A[index];
    } else {
        if (r.GetSupervisorFlag()) {
            //--r.SSP;
            return r.SSP;
        } else {
            //--r.USP;
            return r.USP;
        }
        //return r.GetSupervisorFlag() ? r.SSP : r.USP;
    }
}

} // namespace

void TTarget::SetDataRegister(uint8_t index) {
    Kind_ = DataRegisterKind;
    Value_.DataRegisterIndex = index;
}

void TTarget::SetDecAddressRegisterKind(uint8_t index) {
    Kind_ = DecAddressRegisterKind;
    Value_.DecAddressRegisterIndex = index;
}

TDataHolder TTarget::Read(NEmulator::TContext ctx, TAddressType size) {
    switch (Kind_) {
        case DataRegisterKind: {
            auto reg = ctx.Registers.D[Value_.DataRegisterIndex];
            TDataHolder data;
            for (int i = 0; i < size; ++i) {
                data.push_back(reg & 0xFF);
                reg >>= 8;
            }
            // TODO: reverse `data`?
            return data;
        }
        case DecAddressRegisterKind: {
            auto& reg = GetAReg(ctx.Registers, Value_.DecAddressRegisterIndex);
            --reg;
            if (Value_.DecAddressRegisterIndex == 7) {
                --reg;
            }
            return ctx.Memory.Read(reg, size);
        }
    }
    __builtin_unreachable();
}

TByte TTarget::ReadByte(NEmulator::TContext ctx) {
    return Read(ctx, /*size=*/1)[0];
}

TWord TTarget::ReadWord(NEmulator::TContext ctx) {
    const auto data = Read(ctx, /*size=*/2);
    TWord res = data[0];
    res = (res << 8) + data[1];
    return res;
}

TLong TTarget::ReadLong(NEmulator::TContext ctx) {
    const auto data = Read(ctx, /*size=*/4);
    TLong res = data[0];
    res = (res << 8) + data[1];
    res = (res << 8) + data[2];
    res = (res << 8) + data[3];
    return res;
}

void TTarget::Write(NEmulator::TContext ctx, TDataView data) {
    switch (Kind_) {
        case DataRegisterKind: {
            int shift = 0;
            int lsb = 0;
            for (const auto value : data) {
                shift += 8;
                lsb <<= 8;
                lsb += value;
            }

            auto& reg = ctx.Registers.D[Value_.DataRegisterIndex];
            reg >>= shift;
            reg <<= shift;
            reg += lsb;
            break;
        }
        case DecAddressRegisterKind: {
            const auto reg = GetAReg(ctx.Registers, Value_.DecAddressRegisterIndex);
            ctx.Memory.Write(reg, data);
            break;
        }
    }
}

void TTarget::WriteByte(NEmulator::TContext ctx, TByte b) {
    TDataHolder data;
    data.emplace_back(b);
    return Write(ctx, data);
}

void TTarget::WriteWord(NEmulator::TContext ctx, TWord w) {
    TDataHolder data;
    for (int i = 0; i < 2; ++i) {
        data.emplace_back(w & 0xFF);
        w >>= 8;
    }
    return Write(ctx, data);
}

void TTarget::WriteLong(NEmulator::TContext ctx, TLong l) {
    TDataHolder data;
    for (int i = 0; i < 4; ++i) {
        data.emplace_back(l & 0xFF);
        l >>= 8;
    }
    return Write(ctx, data);
}

} // namespace NOpcodes
