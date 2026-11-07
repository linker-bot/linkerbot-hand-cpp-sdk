find_package(Catch2 3 QUIET)
if(NOT Catch2_FOUND)
    if(NOT LINKERBOT_ALLOW_DEPENDENCY_DOWNLOADS)
        message(FATAL_ERROR "Catch2 3 is required; install it or provide Catch2_DIR (downloads disabled)")
    endif()
    include(FetchContent)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.8.1
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(Catch2)
endif()

file(GLOB_RECURSE LINKERBOT_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
)

list(FILTER LINKERBOT_TEST_SOURCES EXCLUDE REGEX "/protocol/")
list(FILTER LINKERBOT_TEST_SOURCES EXCLUDE REGEX "/native_adapter/")
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # This hardware performance runner uses posix_spawn, signals and SocketCAN.
    list(FILTER LINKERBOT_TEST_SOURCES EXCLUDE REGEX "/l6/test_angle_frequency_performance\\.cpp$")
endif()

add_executable(tests EXCLUDE_FROM_ALL ${LINKERBOT_TEST_SOURCES}
    "${CMAKE_CURRENT_SOURCE_DIR}/../tools/l6_raw_session.cpp")
target_include_directories(tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../tools")
target_include_directories(tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(tests PRIVATE linkerbot::cpp-sdk Catch2::Catch2)
target_compile_definitions(tests PRIVATE LINKERBOT_TEST_BINARY="$<TARGET_FILE:tests>")
target_compile_definitions(tests PRIVATE LINKERBOT_TEST_CONFIG_DIR="${CMAKE_CURRENT_BINARY_DIR}/config")

add_executable(async-command-probe EXCLUDE_FROM_ALL ../validation/async_command_probe.cpp)
target_link_libraries(async-command-probe PRIVATE linkerbot::cpp-sdk)
add_executable(python-migration-probe EXCLUDE_FROM_ALL ../validation/python_migration_probe.cpp)
target_link_libraries(python-migration-probe PRIVATE linkerbot::cpp-sdk)
add_dependencies(tests async-command-probe python-migration-probe)
find_package(Python3 COMPONENTS Interpreter QUIET)
set(LINKERBOT_PYTHON_SDK_DIR "" CACHE PATH "Optional Python SDK source checkout for cross-language async vectors")
if(Python3_Interpreter_FOUND)
    add_test(NAME release-package-audit COMMAND "${Python3_EXECUTABLE}"
        -m unittest discover -s "${CMAKE_CURRENT_SOURCE_DIR}/../validation"
        -p "test_release_*.py")
    # This integration matrix configures fresh compiler projects. Keep its
    # process budget separate from the fast Python package unit tests.
    add_test(NAME package-version-compatibility COMMAND "${Python3_EXECUTABLE}"
        -m unittest discover -s "${CMAKE_CURRENT_SOURCE_DIR}/../validation"
        -p "test_package_compatibility.py")
    set_tests_properties(package-version-compatibility PROPERTIES TIMEOUT 300)
    set(async_vector_options)
    if(LINKERBOT_PYTHON_SDK_DIR)
        list(APPEND async_vector_options --python-sdk "${LINKERBOT_PYTHON_SDK_DIR}")
    endif()
    add_test(NAME async-behavior-vectors COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/../validation/check_async_vectors.py"
        --cpp-probe "$<TARGET_FILE:async-command-probe>"
        --report "${CMAKE_CURRENT_BINARY_DIR}/async-behavior-report.json"
        ${async_vector_options})
    set_tests_properties(async-behavior-vectors PROPERTIES TIMEOUT 120)
    add_test(NAME python-migration-vectors COMMAND "${Python3_EXECUTABLE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/../validation/check_python_migration.py"
        --cpp-probe "$<TARGET_FILE:python-migration-probe>"
        ${async_vector_options})
    set_tests_properties(python-migration-vectors PROPERTIES TIMEOUT 120)
else()
    if(LINKERBOT_PYTHON_SDK_DIR)
        message(FATAL_ERROR "Cross-language vectors require a Python interpreter")
    endif()
    message(STATUS "Python interpreter unavailable: async behavior vector runner is disabled")
endif()

# Hardware-in-the-loop cases live under [l6]/[o6]/[l20lite]/[l20]/[l30]/[o30] and need a real CAN interface.
# Default ctest only runs FakeTransport integration tests.
add_test(NAME comm COMMAND tests "[comm]")
# Discovery contract and adapter tests are offline: injected enumerators and
# FakeTransport cover platform sources without opening host CAN hardware.
add_test(NAME discovery COMMAND tests "[discovery]")

if(TARGET l6_read_version)
    add_dependencies(tests l6_read_version)
    add_test(NAME l6_read_version_help COMMAND l6_read_version --help)
    add_test(NAME l6_read_version_bad_args COMMAND l6_read_version --invalid)
    set_tests_properties(l6_read_version_bad_args PROPERTIES WILL_FAIL TRUE)
endif()

if(TARGET l6_frequency_benchmark)
    target_compile_definitions(tests PRIVATE
        LINKERBOT_L6_BENCHMARK_BINARY="$<TARGET_FILE:l6_frequency_benchmark>")
    add_dependencies(tests l6_frequency_benchmark)
    add_test(
        NAME l6_frequency_benchmark_help
        COMMAND l6_frequency_benchmark --help
    )
    add_test(
        NAME l6_frequency_benchmark_dry_run
        COMMAND l6_frequency_benchmark --dry-run
                --output "${CMAKE_CURRENT_BINARY_DIR}/l6-benchmark-dry-run.json"
    )
    add_test(
        NAME l6_frequency_benchmark_raw_fake
        COMMAND l6_frequency_benchmark
                --layer fake-socketcan
                --data combined
                --mode polling
                --angle-policy query
                --angle-hz 40
                --tactile-hz 8
                --duration 0.2
                --warmup 0.05
                --round-gap 0
                --output "${CMAKE_CURRENT_BINARY_DIR}/l6-benchmark-raw-fake.json"
    )
    add_test(
        NAME l6_frequency_benchmark_bad_args
        COMMAND l6_frequency_benchmark --layer invalid
    )
    set_tests_properties(l6_frequency_benchmark_bad_args
                         PROPERTIES WILL_FAIL TRUE)
endif()

if(TARGET o6_frequency_benchmark)
    add_dependencies(tests o6_frequency_benchmark)
    add_test(
        NAME o6_frequency_benchmark_help
        COMMAND o6_frequency_benchmark --help
    )
    add_test(
        NAME o6_frequency_benchmark_dry_run
        COMMAND o6_frequency_benchmark --dry-run
                --data combined
                --angle-hz 50
                --tactile-hz 10
                --output "${CMAKE_CURRENT_BINARY_DIR}/o6-benchmark-dry-run.json"
    )
    add_test(
        NAME o6_frequency_benchmark_control_fake
        COMMAND o6_frequency_benchmark --layer fake --mode control
                --angle-policy command-feedback --data angle
                --angle-hz 20 --command-hz 20 --duration 0.2
                --warmup 0.05 --rounds 1 --timeout-ms 100
                --confirm-motion
    )
    add_test(
        NAME o6_frequency_benchmark_adaptive_fake
        COMMAND o6_frequency_benchmark --layer fake --mode control
                --angle-policy adaptive --data angle
                --angle-hz 20 --command-hz 20 --duration 0.2
                --warmup 0.05 --rounds 1 --timeout-ms 100
                --confirm-motion
    )
    add_test(
        NAME o6_frequency_benchmark_bad_args
        COMMAND o6_frequency_benchmark --layer invalid
    )
    set_tests_properties(o6_frequency_benchmark_bad_args
                         PROPERTIES WILL_FAIL TRUE)
endif()

# These vectors link only value headers and pure codecs: no comm I/O library,
# dispatcher, thread startup, device access or runtime implementation.
add_library(linkerbot-protocol STATIC
    ../src/hand/o30/protocol.cpp
    ../src/hand/l30/protocol.cpp)
target_include_directories(linkerbot-protocol PUBLIC ../include)
target_link_libraries(linkerbot-protocol PUBLIC linkerbot::comm-values)
add_executable(protocol-tests protocol/test_vectors.cpp protocol/test_l6_hop_vectors.cpp protocol/test_o6_vectors.cpp protocol/test_l20lite_vectors.cpp protocol/main.cpp)
target_link_libraries(protocol-tests PRIVATE linkerbot-protocol Catch2::Catch2)
add_test(NAME protocol-vectors COMMAND protocol-tests "[protocol]")

# L20 production plans are also compiled without Runtime or I/O implementations.
target_sources(protocol-tests PRIVATE protocol/test_l20_vectors.cpp)

# L30 production request/report plans are exercised without Runtime/I/O linkage.
target_sources(protocol-tests PRIVATE protocol/test_l30_vectors.cpp)

# Paired-source native adapter integration. Installed-comm consumers do not need
# test-only comm sources; those installations are covered by installed-consumer.
if(EXISTS "${LINKERBOT_COMM_SOURCE_DIR}/tests/pcan_fixture.cpp")
    add_library(hand-pcan-fixture SHARED "${LINKERBOT_COMM_SOURCE_DIR}/tests/pcan_fixture.cpp")
    add_executable(adapter-native-tests native_adapter/main.cpp native_adapter/test_pcan_default.cpp)
    target_include_directories(adapter-native-tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(adapter-native-tests PRIVATE linkerbot::cpp-sdk Catch2::Catch2)
    if(MSVC)
        target_compile_options(adapter-native-tests PRIVATE /utf-8)
    endif()
    add_dependencies(adapter-native-tests hand-pcan-fixture)
    add_dependencies(tests adapter-native-tests)
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/adapter-native-config")
    add_test(NAME adapter-native-integration COMMAND ${CMAKE_COMMAND} -E env
        "XDG_CONFIG_HOME=${CMAKE_CURRENT_BINARY_DIR}/adapter-native-config"
        "LINKERBOT_TEST_PCAN_LIBRARY=$<TARGET_FILE:hand-pcan-fixture>"
        $<TARGET_FILE:adapter-native-tests>)
    set_tests_properties(adapter-native-integration PROPERTIES TIMEOUT 60)
endif()
