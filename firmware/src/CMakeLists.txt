idf_component_register(
  SRCS
    main.cpp
    embedded_configs.cpp
    step_engine.cpp
    motion_controller.cpp
    serial_console.cpp
    tmc2209.cpp
  INCLUDE_DIRS .
  REQUIRES robottwin_core driver esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++20>)
