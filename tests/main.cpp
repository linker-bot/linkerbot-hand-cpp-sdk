#define CATCH_CONFIG_RUNNER
#include <catch2/catch_all.hpp> // NOLINT(misc-include-cleaner)
#include <cstdlib>
#include <filesystem>

int main(int argc, char *argv[]) {
    // Fake benchmark subprocesses inherit this directory too. Offline tests
    // must never read or update the user's installed mapping configuration.
    std::filesystem::create_directories(LINKERBOT_TEST_CONFIG_DIR);
#ifdef _WIN32
    _putenv_s("XDG_CONFIG_HOME", LINKERBOT_TEST_CONFIG_DIR);
#else
    setenv("XDG_CONFIG_HOME", LINKERBOT_TEST_CONFIG_DIR, 1);
#endif
    return Catch::Session().run(argc, argv);
}
