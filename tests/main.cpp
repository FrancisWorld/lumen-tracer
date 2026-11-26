#include <cstring>

#include "testing.hpp"

int main(int argc, char** argv) {
    using namespace lumen::testing;
    const char* suite = nullptr;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--suite") == 0 && i + 1 < argc) {
            suite = argv[++i];
        } else {
            std::fprintf(stderr, "usage: lumen_tests [--suite NAME]\n");
            return 2;
        }
    }
    int ran = 0;
    int failed_cases = 0;
    for (const Case& c : registry()) {
        if (suite != nullptr && std::strcmp(c.suite, suite) != 0) {
            continue;
        }
        ++ran;
        const int before = failure_count();
        std::fprintf(stderr, "[ RUN  ] %s.%s\n", c.suite, c.name);
        try {
            c.fn();
        } catch (const RequireFailed&) {
        } catch (const std::exception& e) {
            report_failure(__FILE__, __LINE__, std::string("unexpected exception: ") + e.what());
        }
        if (failure_count() != before) {
            ++failed_cases;
            std::fprintf(stderr, "[ FAIL ] %s.%s\n", c.suite, c.name);
        }
    }
    if (ran == 0) {
        std::fprintf(stderr, "no test cases matched\n");
        return 2;
    }
    std::fprintf(stderr, "%d cases, %d failed\n", ran, failed_cases);
    return failed_cases == 0 ? 0 : 1;
}
