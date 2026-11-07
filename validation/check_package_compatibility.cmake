cmake_minimum_required(VERSION 3.16)
include(CMakePackageConfigHelpers)
set(_generator_options)
foreach(_pair GENERATOR ARCHITECTURE TOOLSET)
    if(VALIDATION_${_pair})
        if(_pair STREQUAL "GENERATOR")
            list(APPEND _generator_options -G "${VALIDATION_${_pair}}")
        elseif(_pair STREQUAL "ARCHITECTURE")
            list(APPEND _generator_options -A "${VALIDATION_${_pair}}")
        else()
            list(APPEND _generator_options -T "${VALIDATION_${_pair}}")
        endif()
    endif()
endforeach()

if(NOT DEFINED HAND_PREFIX OR HAND_PREFIX STREQUAL "")
    message(FATAL_ERROR "HAND_PREFIX must point to an installed linkerbot-hand-cpp prefix")
endif()
get_filename_component(_hand_source "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
if(NOT DEFINED HAND_VERSION)
    file(STRINGS "${_hand_source}/CMakeLists.txt" _project_version_line
        REGEX "^[ ]+VERSION [0-9]+[.][0-9]+[.][0-9]+$")
    list(LENGTH _project_version_line _version_count)
    if(NOT _version_count EQUAL 1)
        message(FATAL_ERROR "Expected one hand project version")
    endif()
    string(REGEX REPLACE "^[ ]+VERSION " "" HAND_VERSION "${_project_version_line}")
endif()

include("${_hand_source}/cmake/TransportVersion.cmake")

set(_root "${CMAKE_CURRENT_BINARY_DIR}/package-compatibility")
file(REMOVE_RECURSE "${_root}")
file(MAKE_DIRECTORY "${_root}/project")
file(WRITE "${_root}/project/CMakeLists.txt"
    "cmake_minimum_required(VERSION 3.16)\n"
    "project(package-compatibility-check LANGUAGES CXX)\n"
    "find_package(linkerbot-hand-cpp ${HAND_VERSION} CONFIG REQUIRED)\n")

function(run_case name prefix should_pass)
    set(_project "${_root}/project")
    if(ARGC GREATER 3)
        set(_project "${_root}/${name}-project")
        file(MAKE_DIRECTORY "${_project}")
        file(WRITE "${_project}/CMakeLists.txt"
            "cmake_minimum_required(VERSION 3.16)\n"
            "project(package-parent-check LANGUAGES CXX)\n"
            "add_library(linkerbot::comm INTERFACE IMPORTED)\n"
            "set_target_properties(linkerbot::comm PROPERTIES LINKERBOT_COMM_VERSION ${ARGV3})\n"
            "find_package(linkerbot-hand-cpp ${HAND_VERSION} CONFIG REQUIRED)\n")
    endif()
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -S "${_project}" -B "${_root}/${name}-build"
            ${_generator_options}
            "-DCMAKE_PREFIX_PATH=${prefix}"
            -DCMAKE_FIND_USE_PACKAGE_REGISTRY=FALSE
            -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=FALSE
            -DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=FALSE
            -DCMAKE_FIND_USE_CMAKE_SYSTEM_PATH=FALSE
        RESULT_VARIABLE _result
        OUTPUT_VARIABLE _stdout
        ERROR_VARIABLE _stderr)
    if(should_pass AND NOT _result EQUAL 0)
        message(FATAL_ERROR "${name} package check failed:\n${_stdout}\n${_stderr}")
    elseif(NOT should_pass AND _result EQUAL 0)
        message(FATAL_ERROR "${name} package check unexpectedly succeeded")
    elseif(NOT should_pass AND ARGC GREATER 3 AND NOT _stderr MATCHES "Requires transport")
        message(FATAL_ERROR "${name} did not reject the preloaded transport target:\n${_stdout}\n${_stderr}")
    endif()
    message(STATUS "${name}: expected configure result observed")
endfunction()

file(GLOB_RECURSE _comm_version_files
    "${HAND_PREFIX}/*linkerbot-comm-config-version.cmake")
list(LENGTH _comm_version_files _count)
if(NOT _count EQUAL 1)
    message(FATAL_ERROR "Expected one installed comm version file, found ${_count}")
endif()
list(GET _comm_version_files 0 _comm_version_file)
file(RELATIVE_PATH _comm_version_relative "${HAND_PREFIX}" "${_comm_version_file}")
get_filename_component(_comm_cmake_relative "${_comm_version_relative}" DIRECTORY)

function(comm_version_case name version should_pass)
    set(_prefix "${_root}/${name}")
    file(COPY "${HAND_PREFIX}/" DESTINATION "${_prefix}")
    # Generate a complete version file, including its major/minor comparisons;
    # replacing PACKAGE_VERSION alone does not model a real release package.
    write_basic_package_version_file("${_prefix}/${_comm_version_relative}"
        VERSION "${version}" COMPATIBILITY SameMinorVersion)
    run_case("${name}" "${_prefix}" "${should_pass}")
endfunction()

string(REPLACE "." ";" _version_parts "${LINKERBOT_TRANSPORT_REQUIRED_VERSION}")
list(GET _version_parts 0 _major)
list(GET _version_parts 1 _minor)
list(GET _version_parts 2 _patch)
math(EXPR _next_patch "${_patch} + 1")
math(EXPR _next_minor "${_minor} + 1")
math(EXPR _previous_minor "${_minor} - 1")

run_case(matching "${HAND_PREFIX}" TRUE)
comm_version_case(minimum "${LINKERBOT_TRANSPORT_REQUIRED_VERSION}" TRUE)
math(EXPR _previous_patch "${_patch} - 1")
comm_version_case(older-patch "${_major}.${_minor}.${_previous_patch}" FALSE)
run_case(installed-parent-matching "${HAND_PREFIX}" TRUE "${LINKERBOT_TRANSPORT_REQUIRED_VERSION}")
run_case(installed-parent-older "${HAND_PREFIX}" FALSE "${_major}.${_minor}.${_previous_patch}")
run_case(installed-parent-newer-patch "${HAND_PREFIX}" TRUE "${_major}.${_minor}.${_next_patch}")
run_case(installed-parent-newer-minor "${HAND_PREFIX}" FALSE "${_major}.${_next_minor}.0")
comm_version_case(newer-patch "${_major}.${_minor}.${_next_patch}" TRUE)
comm_version_case(newer-minor "${_major}.${_next_minor}.0" FALSE)
comm_version_case(older-minor "${_major}.${_previous_minor}.0" FALSE)

file(COPY "${HAND_PREFIX}/" DESTINATION "${_root}/missing-comm")
file(REMOVE_RECURSE "${_root}/missing-comm/${_comm_cmake_relative}")
run_case(missing-comm "${_root}/missing-comm" FALSE)

# The source build must also reject a wrong target preloaded by its caller.
function(source_target_case name version should_pass)
    set(_project "${_root}/${name}-project")
    file(MAKE_DIRECTORY "${_project}")
    file(WRITE "${_project}/CMakeLists.txt"
        "cmake_minimum_required(VERSION 3.16)\n"
        "project(source-target-check LANGUAGES CXX)\n"
        "add_library(linkerbot::comm INTERFACE IMPORTED)\n"
        "set_target_properties(linkerbot::comm PROPERTIES LINKERBOT_COMM_VERSION ${version})\n"
        "add_library(linkerbot::comm-compat INTERFACE IMPORTED)\n"
        "add_subdirectory(\"${_hand_source}\" hand)\n")
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -S "${_project}" -B "${_root}/${name}-build"
            ${_generator_options}
            -DLINKERBOT_BUILD_TESTS=OFF -DLINKERBOT_BUILD_TOOLS=OFF
            -DLINKERBOT_BUILD_EXAMPLES=OFF
        RESULT_VARIABLE _result OUTPUT_VARIABLE _stdout ERROR_VARIABLE _stderr)
    if(should_pass AND NOT _result EQUAL 0)
        message(FATAL_ERROR "${name} failed:\n${_stdout}\n${_stderr}")
    elseif(NOT should_pass AND (_result EQUAL 0 OR NOT _stderr MATCHES "requires linkerbot-comm"))
        message(FATAL_ERROR "${name} did not reject the comm version:\n${_stdout}\n${_stderr}")
    endif()
    message(STATUS "${name}: expected configure result observed")
endfunction()
source_target_case(source-matching "${LINKERBOT_TRANSPORT_REQUIRED_VERSION}" TRUE)
source_target_case(source-older-patch "${_major}.${_minor}.${_previous_patch}" FALSE)
source_target_case(source-newer-patch "${_major}.${_minor}.${_next_patch}" TRUE)
source_target_case(source-newer-minor "${_major}.${_next_minor}.0" FALSE)
source_target_case(source-older-minor "${_major}.${_previous_minor}.0" FALSE)
message(STATUS "linkerbot-hand-cpp package compatibility checks passed")
