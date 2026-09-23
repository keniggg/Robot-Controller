# Build CPython bindings against an isolated, precompiled librealsense core.
# The official SDK build must first configure its dependencies and build only rsutils.
cmake_minimum_required(VERSION 3.8)
project(AliciaIsolatedRealSensePython LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(SDK_SOURCE_DIR "" CACHE PATH "Extracted official librealsense v2.56.5 source")
set(SDK_DEPENDENCY_BUILD_DIR "" CACHE PATH "Configured official SDK build containing rsutils and fetched headers")
set(REALSENSE_SHARED_LIBRARY "" CACHE FILEPATH "Extracted official librealsense2.so.2.56.5")
set(PYTHON_EXECUTABLE /usr/bin/python3.8 CACHE FILEPATH "Python interpreter matching the intended ROS process")
foreach(required SDK_SOURCE_DIR SDK_DEPENDENCY_BUILD_DIR REALSENSE_SHARED_LIBRARY)
    if(NOT EXISTS "${${required}}")
        message(FATAL_ERROR "${required} must name an existing path")
    endif()
endforeach()
if(NOT EXISTS "${SDK_DEPENDENCY_BUILD_DIR}/Release/librsutils.a")
    message(FATAL_ERROR "Build the official SDK's rsutils target before building these bindings")
endif()
set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
# The core remains the official optimized binary. Avoid expensive template LTO here.
set(CMAKE_CXX_FLAGS_RELEASE "-O1 -g0 -DNDEBUG")
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)
add_subdirectory("${SDK_DEPENDENCY_BUILD_DIR}/third-party/pybind11" pybind11)
set(SDK "${SDK_SOURCE_DIR}")
pybind11_add_module(pyrealsense2 MODULE NO_EXTRAS
    "${SDK}/wrappers/python/pyrealsense2.cpp"
    "${SDK}/wrappers/python/c_files.cpp"
    "${SDK}/wrappers/python/pyrs_advanced_mode.cpp"
    "${SDK}/wrappers/python/pyrs_context.cpp"
    "${SDK}/wrappers/python/pyrs_device.cpp"
    "${SDK}/wrappers/python/pyrs_export.cpp"
    "${SDK}/wrappers/python/pyrs_frame.cpp"
    "${SDK}/wrappers/python/pyrs_internal.cpp"
    "${SDK}/wrappers/python/pyrs_options.cpp"
    "${SDK}/wrappers/python/pyrs_pipeline.cpp"
    "${SDK}/wrappers/python/pyrs_processing.cpp"
    "${SDK}/wrappers/python/pyrs_record_playback.cpp"
    "${SDK}/wrappers/python/pyrs_sensor.cpp"
    "${SDK}/wrappers/python/pyrs_types.cpp"
    "${SDK}/wrappers/python/pyrsutil.cpp"
    "${SDK}/common/metadata-helper.cpp"
)
target_include_directories(pyrealsense2 PRIVATE
    "${SDK}" "${SDK}/src" "${SDK}/include"
    "${SDK}/third-party/rsutils/include"
    "${SDK_DEPENDENCY_BUILD_DIR}/third-party/json/include"
    "${SDK_DEPENDENCY_BUILD_DIR}/third-party/pybind11-json/include"
)
target_compile_definitions(pyrealsense2 PRIVATE BUILD_EASYLOGGINGPP
    ELPP_NO_DEFAULT_LOG_FILE ELPP_THREAD_SAFE EASYLOGGINGPP_ASYNC
    PYBIND11_COMPILER_TYPE="_pyrealsense2_abi")
target_link_libraries(pyrealsense2 PRIVATE
    "${REALSENSE_SHARED_LIBRARY}"
    "${SDK_DEPENDENCY_BUILD_DIR}/Release/librsutils.a" pthread)
# Copy the core next to the extension or into its lib/ directory, preserving its SONAME.
set_target_properties(pyrealsense2 PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE INSTALL_RPATH "$ORIGIN;$ORIGIN/lib")
