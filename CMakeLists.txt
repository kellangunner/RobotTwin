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
  src/planning/retime.cpp
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
    src/hardware/hardware_config.cpp
    src/hardware/serial_protocol.cpp
    src/hardware/gripper.cpp
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
    # Keep the heap on a plain (non-resizable) ArrayBuffer.
    #
    # Emscripten 6.0.2 defaults GROWABLE_ARRAYBUFFERS to 1, which auto-detects
    # WebAssembly.Memory.toResizableBuffer() and, where present, backs HEAPU8
    # with a *resizable* ArrayBuffer. But its string decoder still hands
    # TextDecoder a plain `.subarray()` view (getUnsharedTextDecoderView only
    # copies for SharedArrayBuffer), and TextDecoder.decode() rejects views
    # onto resizable buffers:
    #   "The provided ArrayBuffer value must not be resizable"
    #
    # Every std::string longer than 16 bytes crossing to JS therefore threw —
    # shorter ones take the manual decode path and hid the bug. That is exactly
    # our collision messages ("forearm/gripper hits the ground"), so any
    # colliding pose crashed the app on browsers new enough to have the API.
    # Auto-detection also made it invisible in local testing.
    #
    # Revisit once upstream teaches the decoder to copy for resizable buffers.
    -sGROWABLE_ARRAYBUFFERS=0
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
    tests/test_retime.cpp
    tests/test_collision.cpp
    tests/test_waypoints.cpp
    tests/test_serial_protocol.cpp
    tests/test_hardware_config.cpp
    tests/test_gripper.cpp
  )
  target_link_libraries(robottwin_tests PRIVATE robottwin_core)
  target_compile_definitions(robottwin_tests PRIVATE
    ROBOTTWIN_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/robot.yaml"
    ROBOTTWIN_FIRMWARE_CONFIG_PATH="${CMAKE_SOURCE_DIR}/config/firmware.yaml"
  )
  add_test(NAME core COMMAND robottwin_tests)
endif()
