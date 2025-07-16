cmake_minimum_required(VERSION 3.20)

cmake_policy(SET CMP0048 NEW)

# Override the default system to cross-compile for ARM processors
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Set compilation configurations for libraries
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set the boot memory type
set (APP_TYPE BOOT_SRAM)

# Set the path to the libDaisy and DaisySP directories
set(LIBDAISY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/libDaisy)
set(LIBDAISY_CORE_DIR ${LIBDAISY_DIR}/core)
set(DAISYSP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/DaisySP)

# Sources directory
set(APP_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/)

# Include the toolchain files
include (${LIBDAISY_DIR}/cmake/toolchains/autodetect.cmake)
# Include the chip and architecture specific flags
include (${LIBDAISY_DIR}/cmake/toolchains/stm32h750xx.cmake)

# open-ocd -  might need to be adapted for your environment...
set(CHIPSET stm32h7x)
set(OCD openocd)
set(OCD_DIR /usr/local/share/openocd/scripts)
set(PGM_DEVICE interface/stlink.cfg)
set(OCDFLAGS -f ${PGM_DEVICE} -f target/${CHIPSET}.cfg)
set(OCD_PROGRAM ${OCD} -s ${OCD_DIR} ${OCDFLAGS} -c "program ${TARGET}.elf verify reset exit")

# Set the CMake project name and version
project(spotykach_hwtest VERSION 0.0.1 LANGUAGES CXX)
set(TARGET spotykach_hwtest)

# Add the Daisy and DaisySP libraries + the application source files
add_subdirectory(${LIBDAISY_DIR})
add_subdirectory(${DAISYSP_DIR})
add_subdirectory(${APP_SRC_DIR})

add_executable(
  ${TARGET}
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${APP_SRC_DIR}/app.cpp
)

target_include_directories(
    ${TARGET}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src/
      ${CMAKE_CURRENT_SOURCE_DIR}/lib/
)

target_link_libraries(
  ${TARGET}
  PUBLIC
    daisy
    DaisySP
    hwtest
)

set (INTERNAL_ADDRESS 0x08000000)
set (QSPI_ADDRESS 0x90040000)
set (SRAM_ADDRESS 0x24000000)

# For the time being, we'll keep the Daisy Bootloader's PID as df11
# DAISY_PID = a360
set (DAISY_PID df11)
set (STM_PID df11)


if (${APP_TYPE} STREQUAL "BOOT_NONE")
  set (LINKER_SCRIPT ${LIBDAISY_CORE_DIR}/STM32H750IB_flash.lds)
  set (USBPID $(STM_PID))
  set (FLASH_ADDRESS $(INTERNAL_ADDRESS))
elseif(${APP_TYPE} STREQUAL "BOOT_SRAM")
  # set (LINKER_SCRIPT ${LIBDAISY_CORE_DIR}/STM32H750IB_sram.lds)
  set (LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/alt_sram.lds)
  set (USBPID $(DAISY_PID))
  set (FLASH_ADDRESS $(QSPI_ADDRESS))
  set (BOOT_DEFS -DBOOT_APP)
elseif(${APP_TYPE} STREQUAL "BOOT_QSPI")
  set (LINKER_SCRIPT ${LIBDAISY_CORE_DIR}/STM32H750IB_qspi.lds)
  set (USBPID $(DAISY_PID))
  set (FLASH_ADDRESS $(QSPI_ADDRESS))
  set (BOOT_DEFS -DBOOT_APP)
else()
  message(FATAL_ERROR "Unkown app type ${APP_TYPE}")
endif()

# Global optimization options
if(NOT DEFINED FIRMWARE_DEBUG_OPT_LEVEL)
    set(FIRMWARE_DEBUG_OPT_LEVEL -Og)
endif()

if(NOT DEFINED FIRMWARE_RELEASE_OPT_LEVEL)
    set(FIRMWARE_RELEASE_OPT_LEVEL -O3)
endif()

if(NOT DEFINED FIRMWARE_MINSIZEREL_OPT_LEVEL)
    # Optimize for size. -Os enables all -O2 optimizations.
    set(FIRMWARE_MINSIZE_REL_OPT_LEVEL -Os)
endif()

set_target_properties(
  ${TARGET}
  PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    SUFFIX ".elf"
    LINK_DEPENDS ${LINKER_SCRIPT}
)

# Set the compile options, including settings dependent on the build configuration
target_compile_options(
  ${TARGET}
  PUBLIC
    -Wall
    -Wno-attributes
    -Wno-strict-aliasing
    -Wno-maybe-uninitialized
    -Wno-missing-attributes
    -Wno-stringop-overflow
    -Wno-error=reorder
    -Wno-error=sign-compare
    -fexceptions
    -DQ_DONT_USE_THREADS=1
    -DUSE_HAL_DRIVER
    -DUSE_FULL_LL_DRIVER
    ${BOOT_DEFS}
    # User flags
    -ffast-math
    -funroll-loops
    -DDDATA_IN_D2_SRAM
    -DINFS_LOG_TARGET=daisy::LOGGER_EXTERNAL
    -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
    $<$<CONFIG:Debug>:${FIRMWARE_DEBUG_OPT_LEVEL}>
    $<$<CONFIG:Release>:${FIRMWARE_RELEASE_OPT_LEVEL}>
    $<$<CONFIG:MinSizeRel>:${FIRMWARE_MINSIZEREL_OPT_LEVEL}>
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-register>
    $<$<CONFIG:Release,MinSizeRel,RelWithDebInfo>:-flto>
    $<$<CONFIG:Debug,RelWithDebInfo>:-ggdb3>
    $<$<CONFIG:Release,MinSizeRel>:-DNDEBUG=1>
    $<$<CONFIG:Debug,>:-DDEBUG=1>
)

target_link_options(
  ${TARGET}
  PUBLIC
    -T ${LINKER_SCRIPT}
    -Wl,-Map=${TARGET}.map,--cref
    -Wl,--check-sections
    -Wl,--unresolved-symbols=report-all
    -Wl,--warn-common
    -Wl,--warn-section-align
    -Wl,--print-memory-usage
)

add_custom_command(
  TARGET ${TARGET}
  POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
    ARGS -O ihex
    -S ${TARGET}.elf
    ${TARGET}.hex
  BYPRODUCTS
    ${TARGET}.hex
    COMMENT "Generating HEX image"
VERBATIM)

add_custom_command(
  TARGET ${TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY}
    ARGS -O binary
    -S ${TARGET}.elf
    ${TARGET}.bin
    BYPRODUCTS
    ${TARGET}.bin
    COMMENT "Generating binary image"
VERBATIM)

add_custom_target(program
  COMMAND ${OCD_PROGRAM}
)
