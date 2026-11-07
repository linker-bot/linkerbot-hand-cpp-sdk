cmake_minimum_required(VERSION 3.16)

project(linkerbot-hand-cpp
    VERSION 0.3.6
    DESCRIPTION "LinkerBot dexterous hand C++ SDK"
    LANGUAGES CXX
)
if(MSVC)
    add_compile_options(/utf-8)
    if(NOT DEFINED CMAKE_MSVC_RUNTIME_LIBRARY)
        set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
    endif()
endif()

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(LINKERBOT_IS_TOP_LEVEL ON)
else()
    set(LINKERBOT_IS_TOP_LEVEL OFF)
endif()

option(LINKERBOT_BUILD_EXAMPLES "Build example programs" ${LINKERBOT_IS_TOP_LEVEL})
option(LINKERBOT_BUILD_TESTS "Build tests" ${LINKERBOT_IS_TOP_LEVEL})
option(LINKERBOT_BUILD_TOOLS "Build developer tools" ${LINKERBOT_IS_TOP_LEVEL})

if(LINKERBOT_IS_TOP_LEVEL AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(LINKERBOT_IS_TOP_LEVEL)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(LINKERBOT_ALLOW_DEPENDENCY_DOWNLOADS "Allow missing test dependencies to be fetched" ON)
include(cmake/TransportVersion.cmake)
include(cmake/TransportDependency.cmake)

# Check both paired sources and a target supplied by the parent project.
get_target_property(_linkerbot_comm_version linkerbot::comm LINKERBOT_COMM_VERSION)
string(REGEX MATCH "^[0-9]+\\.[0-9]+" _linkerbot_comm_line "${_linkerbot_comm_version}")
if(NOT _linkerbot_comm_line STREQUAL "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
   OR _linkerbot_comm_version VERSION_LESS LINKERBOT_TRANSPORT_REQUIRED_VERSION)
    message(FATAL_ERROR
        "linkerbot-hand-cpp ${PROJECT_VERSION} requires linkerbot-comm "
        "${LINKERBOT_TRANSPORT_REQUIRED_VERSION} or newer in the same minor line; found '${_linkerbot_comm_version}'. "
        "Update the paired comm checkout or installed package.")
endif()

file(GLOB_RECURSE LINKERBOT_SDK_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(linkerbot-cpp-sdk STATIC ${LINKERBOT_SDK_SOURCES})
add_library(linkerbot::cpp-sdk ALIAS linkerbot-cpp-sdk)

target_include_directories(linkerbot-cpp-sdk
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_compile_features(linkerbot-cpp-sdk PUBLIC cxx_std_17)
# comm-compat keeps flat includes (`"can_dispatcher.hpp"`) working for
# in-flight consumers; canonical includes are <linkerbot/can/...>.
target_link_libraries(linkerbot-cpp-sdk
    PUBLIC
        Threads::Threads
        linkerbot::comm-compat
)

if(LINKERBOT_BUILD_TESTS)
    enable_testing()
endif()

if(LINKERBOT_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(LINKERBOT_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

if(LINKERBOT_BUILD_TESTS)
    add_subdirectory(tests)
endif()

set_target_properties(linkerbot-cpp-sdk PROPERTIES EXPORT_NAME cpp-sdk)
install(TARGETS linkerbot-cpp-sdk EXPORT linkerbot-hand-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT linkerbot-hand-targets NAMESPACE linkerbot::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/linkerbot-hand-cpp)
configure_package_config_file(cmake/linkerbot-hand-cpp-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/linkerbot-hand-cpp-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/linkerbot-hand-cpp)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/linkerbot-hand-cpp-config-version.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMinorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/linkerbot-hand-cpp-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/linkerbot-hand-cpp-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/linkerbot-hand-cpp)
install(FILES LICENSE README.md README.en.md README.zh-CN.md NOTICE
    CHANGELOG.md CHANGELOG.zh-CN.md CONTRIBUTING.md CONTRIBUTING.zh-CN.md
    DESTINATION ${CMAKE_INSTALL_DATADIR}/linkerbot-hand-cpp
    COMPONENT Documentation)
install(FILES docs/README.md docs/README.en.md docs/README.zh-CN.md
    DESTINATION ${CMAKE_INSTALL_DATADIR}/linkerbot-hand-cpp/docs
    COMPONENT Documentation)
install(FILES docs/quality-baseline-2026-11-03.csv
    DESTINATION ${CMAKE_INSTALL_DATADIR}/linkerbot-hand-cpp/docs
    COMPONENT Documentation)
install(FILES .github/cppreportcard/README.md .github/cppreportcard/README.zh-CN.md
    .github/cppreportcard/badge.svg
    DESTINATION ${CMAKE_INSTALL_DATADIR}/linkerbot-hand-cpp/.github/cppreportcard
    COMPONENT Documentation)
install(DIRECTORY docs/zh docs/en docs/hardware-acceptance
    DESTINATION ${CMAKE_INSTALL_DATADIR}/linkerbot-hand-cpp/docs
    COMPONENT Documentation
    FILES_MATCHING PATTERN "*.md")
install(FILES tests/hand/device/hardware/README.md tests/hand/device/hardware/README.zh-CN.md
    DESTINATION ${CMAKE_INSTALL_DATADIR}/linkerbot-hand-cpp/tests/hand/device/hardware
    COMPONENT Documentation)
install(FILES validation/installed-consumer/CMakeLists.txt validation/installed-consumer/main.cpp
    DESTINATION ${CMAKE_INSTALL_DATADIR}/linkerbot-hand-cpp/validation/installed-consumer)
install(FILES validation/fake_classic_hand.hpp
    DESTINATION ${CMAKE_INSTALL_DATADIR}/linkerbot-hand-cpp/validation)

include(cmake/ReleasePackage.cmake)

if(LINKERBOT_IS_TOP_LEVEL)
    include(CPack)
endif()
