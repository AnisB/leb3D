# This module is shared; use include blocker.
if( _PLATFORMS_ )
	return()
endif()

# Mark it as processed
set(_PLATFORMS_ 1)

# Detect target platform
set(PLATFORM_WINDOWS 1)
set(PLATFORM_NAME "windows")
add_definitions(-DWINDOWSPC)

message(STATUS "Detected platform: ${PLATFORM_NAME}")

# Detect target architecture
if(PLATFORM_WINDOWS AND CMAKE_CL_64)
	set(PLATFORM_64BIT 1)
endif()

# Configure CMake global variables
set(CMAKE_INSTALL_MESSAGE LAZY)

# Set the output folders based on the identifier
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/output/bin)
set(CMAKE_VS_INCLUDE_INSTALL_TO_DEFAULT_BUILD 1)

# Find D3D12 and enable it if possible
FIND_PACKAGE(D3D12)
if (D3D12_FOUND)
	add_definitions(-DD3D12_SUPPORTED)
	add_definitions(-DDX12_SDK_VERSION=717)
endif()

# OpenMP required for the parallel computation
find_package(OpenMP REQUIRED)