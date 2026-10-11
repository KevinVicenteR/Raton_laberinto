# Simulador y pruebas en PC. El firmware del robot se compila con PlatformIO (platformio.ini).
#
#   cmake -B build
#   cmake --build build
#   ctest --test-dir build
cmake_minimum_required(VERSION 3.16)
project(raton_laberinto CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
  add_compile_options(/W4 /utf-8)
else()
  add_compile_options(-Wall -Wextra)
endif()

# Logica comun: es la misma carpeta que usa el firmware.
file(GLOB RATON_CORE_SOURCES CONFIGURE_DEPENDS lib/raton_core/src/*.cpp)
add_library(raton_core STATIC ${RATON_CORE_SOURCES})
target_include_directories(raton_core PUBLIC lib/raton_core/src)

add_library(raton_simlib STATIC sim/maze_io.cpp sim/sim_robot.cpp)
target_include_directories(raton_simlib PUBLIC sim)
target_link_libraries(raton_simlib PUBLIC raton_core)

add_executable(raton_sim sim/main.cpp)
target_link_libraries(raton_sim PRIVATE raton_simlib)

add_executable(raton_mms sim/mms_main.cpp)
target_link_libraries(raton_mms PRIVATE raton_core)

enable_testing()
add_executable(raton_tests tests/test_core.cpp)
target_link_libraries(raton_tests PRIVATE raton_simlib)
target_compile_definitions(raton_tests PRIVATE RATON_MAZES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/mazes")
add_test(NAME core COMMAND raton_tests)
