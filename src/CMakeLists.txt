cmake_minimum_required(VERSION 3.25)
project(EDACup2025 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Find required packages
find_package(nlohmann_json CONFIG REQUIRED)

# --- Main executable (Strategy) ---
add_executable(strategy
    main.cpp
    geometry.cpp
    actions.cpp
    strategy.cpp
    utils.cpp
)

target_link_libraries(strategy PRIVATE nlohmann_json::nlohmann_json)

# --- Coordinate test executable ---
add_executable(test_coords
    test_coords.cpp
)

target_link_libraries(test_coords PRIVATE nlohmann_json::nlohmann_json)

# Optional: Set output directory
set_target_properties(strategy test_coords
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Optional: Add compile warnings for better code quality
target_compile_options(strategy PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

target_compile_options(test_coords PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)