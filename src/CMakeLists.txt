cmake_minimum_required(VERSION 3.14)

project(m68k-emu VERSION 0.1 LANGUAGES CXX)

# compiler settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_compile_options(-g)
add_compile_options(-fno-exceptions)
add_compile_options(-fsanitize=address)
add_link_options(-fsanitize=address)

# add subdirectories
add_subdirectory(lib)

# enable googletest
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.0
)
FetchContent_MakeAvailable(googletest)
enable_testing()
