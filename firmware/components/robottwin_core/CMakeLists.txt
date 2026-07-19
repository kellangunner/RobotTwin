# The repository's C++20 core, unchanged, as an ESP-IDF component — the same
# sources that power the native tests and the WASM twin. Also embeds both YAML
# configs into the image so the robot boots from the twin's source of truth.
get_filename_component(RT_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../.. ABSOLUTE)

idf_component_register(
  SRCS
    ${RT_ROOT}/src/kinematics/kinematics.cpp
    ${RT_ROOT}/src/drivetrain/drivetrain.cpp
    ${RT_ROOT}/src/drivetrain/gearbox_model.cpp
    ${RT_ROOT}/src/dynamics/dynamics.cpp
    ${RT_ROOT}/src/planning/trajectory.cpp
    ${RT_ROOT}/src/planning/retime.cpp
    ${RT_ROOT}/src/simulation/metrics.cpp
    ${RT_ROOT}/src/geometry/collision.cpp
    ${RT_ROOT}/src/io/waypoints.cpp
    ${RT_ROOT}/src/config/yaml_lite.cpp
    ${RT_ROOT}/src/config/load.cpp
    ${RT_ROOT}/src/hardware/hardware_config.cpp
    ${RT_ROOT}/src/hardware/serial_protocol.cpp
  INCLUDE_DIRS ${RT_ROOT}/src
  EMBED_TXTFILES
    ${RT_ROOT}/config/robot.yaml
    ${RT_ROOT}/config/firmware.yaml
)

# IDF picks its own default C++ standard; the core requires C++20.
target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++20>)
