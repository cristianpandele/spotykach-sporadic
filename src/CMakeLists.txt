set(TARGET hwtest)

add_library(
    ${TARGET}
    app.cpp
    color.cpp
    hardware.cpp
    sr_165.cpp
    ws2812.cpp
)

set(LIBDAISY_DIR ${CMAKE_SOURCE_DIR}/lib/libDaisy)
set(DAISYSP_DIR ${CMAKE_SOURCE_DIR}/lib/DaisySP)

target_include_directories(
    ${TARGET}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LIBDAISY_DIR}/Drivers/CMSIS/DSP/Include
    ${LIBDAISY_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include/
    ${DAISYSP_DIR}/Source
)

set (CPP_FLAGS
    -fno-exceptions
    -fasm
    -finline
    -finline-functions-called-once
    -fshort-enums
    -fno-move-loop-invariants
    -fno-unwind-tables
    -fno-rtti
    -g
    -ggdb
    -MMD
    -MP
    # -MF ${CMAKE_CURRENT_BINARY_DIR}/$<TARGET_OBJECTS:${TARGET}>.dep
)

target_compile_options(
  ${TARGET}
  PRIVATE
    -O2
    -Wall
    -Wno-missing-attributes
    -Wno-stringop-overflow
    ${CPP_FLAGS}
    -DUSE_HAL_DRIVER
    -DUSE_FULL_LL_DRIVER
    -DBOOT_APP
    -DINFS_LOG_TARGET=daisy::LOGGER_EXTERNAL
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-register>
)

target_link_libraries(
    ${TARGET}
    PRIVATE
    daisy
)