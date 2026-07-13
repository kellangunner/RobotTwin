cmake_minimum_required(VERSION 3.20)
project(robottwin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------- core library
add_library(robottwin_core STATIC
  src/kinematics/kinematics.cpp
  src/drivetrain/drivetrain.cpp
  src/drivetrain/gearbox_model.cpp
  src/dynamics/dynamics.cpp
  src/planning/trajectory.cpp
  src/simulation/metrics.cpp
  src/geometry/collision.cpp
  src/io/waypoints.cpp
)
target_include_directories(robottwin_core PUBLIC src)

if(MSVC)
  target_compile_options(robottwin_core PUBLIC /W4 /utf-8)
else()
  target_compile_options(robottwin_core PUBLIC -Wall -Wextra)
endif()

# YAML loading is native-only; the WASM build receives parsed config from JS.
if(NOT EMSCRIPTEN)
  target_sources(robottwin_core PRIVATE
    src/config/yaml_lite.cpp
    src/config/load.cpp
  )
endif()

# ------------------------------------------------------------------ wasm build
# Configure with the Emscripten toolchain to produce the web module:
#   emcmake cmake -S . -B build-wasm && cmake --build build-wasm
if(EMSCRIPTEN)
  add_executable(robottwin_wasm src/wasm/bindings.cpp)
  target_link_libraries(robottwin_wasm PRIVATE robottwin_core)
  target_link_options(robottwin_wasm PRIVATE
    --bind
    -sMODULARIZE=1
    -sEXPORT_ES6=1
    -sEXPORT_NAME=createRobotTwinModule
    -sALLOW_MEMORY_GROWTH=1
    -sENVIRONMENT=web
  )
  set_target_properties(robottwin_wasm PROPERTIES
    OUTPUT_NAME "robottwin"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/web/src/wasm"
  )
endif()

# ----------------------------------------------------------------------- tests
if(NOT EMSCRIPTEN)
  enable_testing()
  add_executable(robottwin_tests
    tests/test_main.cpp
    tests/test_kinematics.cpp
    tests/test_drivetrain.cpp
    tests/test_gearbox_model.cpp
    tests/test_trajectory.cpp
    tests/test_collision.cpp
    tests/test_waypoints.cpp
  )
  target_link_libraries(robottwin_tests PRIVATE robottwin_core)
  target_compile_definitions(robottwin_tests PRIVATE
    ROBOTTWIN_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/robot.yaml"
  )
  add_test(NAME core COMMAND robottwin_tests)
endif()
