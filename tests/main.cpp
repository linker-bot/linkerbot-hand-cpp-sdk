#define CATCH_CONFIG_RUNNER
#include <catch2/catch_all.hpp> // NOLINT(misc-include-cleaner)
#include <cstdlib>
#include <filesystem>

#ifdef LINKERBOT_DIAGNOSE_WINDOWS_FAULTS
#include <cstdio>
#include <windows.h>
namespace {
    LONG CALLBACK trace_fault(EXCEPTION_POINTERS *fault) {
        if (fault->ExceptionRecord->ExceptionCode != EXCEPTION_ACCESS_VIOLATION &&
            fault->ExceptionRecord->ExceptionCode != EXCEPTION_STACK_OVERFLOW) {
            return EXCEPTION_CONTINUE_SEARCH;
        }
        std::fprintf(stderr, "FAULT code=%08lx address=%p\n", fault->ExceptionRecord->ExceptionCode,
                     fault->ExceptionRecord->ExceptionAddress);
        void *frames[64]{};
        const auto count = CaptureStackBackTrace(0, 64, frames, nullptr);
        for (USHORT index = 0; index < count; ++index) {
            HMODULE module = nullptr;
            char path[MAX_PATH]{};
            GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                               reinterpret_cast<LPCSTR>(frames[index]), &module);
            GetModuleFileNameA(module, path, MAX_PATH);
            std::fprintf(stderr, "FRAME %u %s+%llx\n", index, path,
                         static_cast<unsigned long long>(reinterpret_cast<uintptr_t>(frames[index]) -
                                                         reinterpret_cast<uintptr_t>(module)));
        }
        std::fflush(stderr);
        return EXCEPTION_CONTINUE_SEARCH;
    }
}
#endif

int main(int argc, char *argv[]) {
#ifdef LINKERBOT_DIAGNOSE_WINDOWS_FAULTS
    AddVectoredExceptionHandler(1, trace_fault);
#endif
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
