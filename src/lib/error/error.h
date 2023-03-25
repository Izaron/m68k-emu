#pragma once

#include <string>

class TError {
public:
    enum EKind {
        NoError,
        UnknownOpcode,
        UnknownAddressingMode,
        UnalignedMemoryAccess,
        UnalignedProgramCounter,
    };

    TError() = default;
    TError(EKind kind, const char* format, ...);

    EKind GetKind() const { return Kind_; }
    const std::string& GetWhat() const { return What_; }

private:
    EKind Kind_ = NoError;
    std::string What_;
};
