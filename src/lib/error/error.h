#pragma once

#include <string>

class TError {
public:
    enum EKind {
        NoError,
        UnknownOpcode,
        UnknownAddressingMode,
        UnalignedMemoryAccess,
    };

    TError(EKind kind, const char* format, ...);

    EKind GetKind() const { return Kind_; }
    const std::string& GetWhat() const { return What_; }

private:
    EKind Kind_;
    std::string What_;
};
