cmake_minimum_required(VERSION 3.25)
project(EDACup2025 LANGUAGES CXX)

# Configuración del estándar C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Flags de optimización
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")

# Encontrar paquetes requeridos
find_package(nlohmann_json CONFIG REQUIRED)

# ============================================================================
# Ejecutable principal (Strategy)
# ============================================================================

# Archivos fuente
set(MAIN_SOURCES
    main.cpp
    geometry.cpp
    robot.cpp
    gamestate.cpp
    ball.cpp
)

# Archivos header (para detección de cambios)
set(MAIN_HEADERS
    robot.h
    geometry.h
    gamestate.h
    constants.h
    ball.cpp
)

add_executable(main ${MAIN_SOURCES} ${MAIN_HEADERS})

target_link_libraries(main PRIVATE nlohmann_json::nlohmann_json)

# ============================================================================
# Ejecutable de prueba (Coordinate test)
# ============================================================================

add_executable(test_coords test_coords.cpp)

target_link_libraries(test_coords PRIVATE nlohmann_json::nlohmann_json)

# ============================================================================
# Configuración de salida
# ============================================================================

set_target_properties(main test_coords
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# ============================================================================
# Warnings del compilador
# ============================================================================

target_compile_options(main PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

target_compile_options(test_coords PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# ============================================================================
# Información de compilación
# ============================================================================

message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Strategy sources: ${STRATEGY_SOURCES}")
message(STATUS "Strategy headers: ${STRATEGY_HEADERS}")