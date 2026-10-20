# Nucleo portable, simulador y pruebas en el PC. El firmware se compila con PlatformIO (platformio.ini).
#
#   cmake -S . -B build && cmake --build build -j
#   ctest --test-dir build --output-on-failure
#   build/raton_sim --bench 20
cmake_minimum_required(VERSION 3.16)
project(raton_laberinto CXX)

set(CMAKE_CXX_STANDARD 14)  # como el nucleo mbed de Arduino
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

option(RATON_SANITIZE "Compilar con AddressSanitizer y UBSan" OFF)
if(RATON_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

file(GLOB_RECURSE RATON_CORE_SOURCES CONFIGURE_DEPENDS lib/raton_core/src/*.cpp)
add_library(raton_core STATIC ${RATON_CORE_SOURCES})
target_include_directories(raton_core PUBLIC lib/raton_core/src)
# Trazas de depuracion por variables de entorno (NAVDBG, VOTEDBG, RADARDBG) solo en el PC.
target_compile_definitions(raton_core PRIVATE RATON_DEBUG)

add_library(raton_simlib STATIC sim/sim.cpp sim/run_case.cpp)
target_include_directories(raton_simlib PUBLIC sim)
target_link_libraries(raton_simlib PUBLIC raton_core)

add_executable(raton_sim sim/main.cpp)
target_link_libraries(raton_sim PRIVATE raton_simlib)
target_compile_definitions(raton_sim PRIVATE RATON_MAZES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/mazes")

enable_testing()
add_executable(raton_tests tests/test_main.cpp)
target_link_libraries(raton_tests PRIVATE raton_simlib)
target_compile_definitions(raton_tests PRIVATE RATON_MAZES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/mazes")
add_test(NAME nucleo COMMAND raton_tests)

# Comprobacion del firmware en el PC: compila todos los .cpp de firmware/ contra cabeceras minimas de
# Arduino/mbed (tests/stubs). No se enlaza ni se ejecuta; detecta errores de tipos y sintaxis sin la placa.
file(GLOB_RECURSE RATON_FW_SOURCES CONFIGURE_DEPENDS firmware/*.cpp)
add_library(firmware_check OBJECT ${RATON_FW_SOURCES})
target_include_directories(firmware_check PRIVATE firmware tests/stubs lib/raton_core/src)
target_compile_options(firmware_check PRIVATE -Wno-unused-parameter)
