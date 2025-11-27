cmake_minimum_required(VERSION 3.10)
project(piper_sdk_release VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Detect system architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(ARCH_DIR "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(ARCH_DIR "x86_64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(PIPER_SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(PIPER_SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH_DIR})

# Check library files
if(EXISTS "${PIPER_SDK_LIB_DIR}/libpiper_sdk.so")
    set(PIPER_SDK_SHARED_LIB "${PIPER_SDK_LIB_DIR}/libpiper_sdk.so")
    set(USE_SHARED_LIB TRUE)
elseif(EXISTS "${PIPER_SDK_LIB_DIR}/libpiper_sdk.a")
    set(PIPER_SDK_STATIC_LIB "${PIPER_SDK_LIB_DIR}/libpiper_sdk.a")
    set(USE_STATIC_LIB TRUE)
else()
    message(WARNING "No library files found in: ${PIPER_SDK_LIB_DIR}")
    return()
endif()

include_directories(${PIPER_SDK_INCLUDE_DIR})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Build all examples
file(GLOB EXAMPLE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/examples/*.cpp)
list(FILTER EXAMPLE_SOURCES EXCLUDE REGEX ".*README.*")

foreach(EXAMPLE_SOURCE ${EXAMPLE_SOURCES})
    get_filename_component(EXAMPLE_NAME ${EXAMPLE_SOURCE} NAME_WE)
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_SOURCE})
    
    if(USE_SHARED_LIB)
        target_link_libraries(${EXAMPLE_NAME} ${PIPER_SDK_SHARED_LIB})
    elseif(USE_STATIC_LIB)
        target_link_libraries(${EXAMPLE_NAME} ${PIPER_SDK_STATIC_LIB})
    endif()
    
    set_target_properties(${EXAMPLE_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
    )
endforeach()


