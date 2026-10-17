# Pruebas del nucleo portable en el PC y biblioteca nativa del simulador Python.
# El firmware del robot se compila con PlatformIO (platformio.ini).
#
#   cmake -S . -B build
#   cmake --build build
#   ctest --test-dir build --output-on-failure
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

option(RATON_SANITIZE "Compilar las pruebas con AddressSanitizer y UBSan" OFF)

# Nucleo comun: la misma carpeta que compila el firmware (lib/raton_core).
file(GLOB_RECURSE RATON_CORE_SOURCES CONFIGURE_DEPENDS lib/raton_core/src/*.cpp)
add_library(raton_core STATIC ${RATON_CORE_SOURCES})
target_include_directories(raton_core PUBLIC lib/raton_core/src)

enable_testing()
add_executable(raton_tests tests/cpp/test_core.cpp)
target_link_libraries(raton_tests PRIVATE raton_core)
target_compile_definitions(raton_tests PRIVATE RATON_MAZES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/mazes")
if(RATON_SANITIZE AND NOT MSVC)
  target_compile_options(raton_core PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_compile_options(raton_tests PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(raton_tests PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME nucleo COMMAND raton_tests)

# Verificador de barrido de giros para el simulador Python (ctypes, variable RATON_NATIVE_LIB).
add_library(raton_navigation SHARED simulator/native/navigation_bridge.cpp simulator/native/motion_safety.cpp)
target_include_directories(raton_navigation PRIVATE simulator/native)
