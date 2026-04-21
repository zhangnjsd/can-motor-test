cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME can-motor-test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB_RECURSE MYBSP_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Mybsp/*.c
)
# Exclude bsp_dt7.c due to missing huart3 definition
list(REMOVE_ITEM MYBSP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Mybsp/bsp_dt7.c")

file(GLOB_RECURSE MYMIDDLEWARE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Mymiddlewares/*.c
)
# Exclude middleware sources with missing dependencies
list(REMOVE_ITEM MYMIDDLEWARE_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/Mymiddlewares/NUC_receive.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Mymiddlewares/kalman_filter_task.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Mymiddlewares/remote_control.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Mymiddlewares/CAN_receive.c"
)
file(GLOB_RECURSE ALGORITHM_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/*.c
)
list(REMOVE_ITEM ALGORITHM_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/Algorithm/SolveTrajectory.c"
)

file(GLOB_RECURSE RTOS_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/*.c
)
list(REMOVE_ITEM RTOS_SOURCES 
    # Re-enable INS_task_fun.c for IMU support
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${MYBSP_SOURCES}
    ${MYMIDDLEWARE_SOURCES}
    ${ALGORITHM_SOURCES}
    ${RTOS_SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/Mydrive/BMI088driver.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Mybsp/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Mymiddlewares
    ${CMAKE_CURRENT_SOURCE_DIR}/Algorithm
    ${CMAKE_CURRENT_SOURCE_DIR}/Mydrive/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Application/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    ARM_MATH_CM4
    ARM_MATH_MATRIX_CHECK
    ARM_MATH_ROUNDING
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
