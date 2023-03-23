add_library(opcodes STATIC
    instructions.cpp
    targets.cpp
)

target_link_libraries(opcodes registers)
