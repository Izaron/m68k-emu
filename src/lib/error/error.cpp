#include "error.h"

#include <cstdarg>

TError::TError(EKind kind, const char* format, ...)
    : Kind_{kind}
{
    char buffer[256];
    std::va_list args;
    va_start(args, format);
    vsnprintf(buffer, 256, format, args);
    va_end(args);

    What_ = std::string{buffer};
}
