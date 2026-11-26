// Minimal test harness: self-registering cases grouped in suites, non-fatal
// CHECKs and fatal REQUIREs. Kept in-tree so the build needs no downloads.
#pragma once

#include <cmath>
#include <cstdio>
#include <exception>
#include <sstream>
#include <string>
#include <vector>

namespace lumen::testing {

struct Case {
    const char* suite;
    const char* name;
    void (*fn)();
};

inline std::vector<Case>& registry() {
    static std::vector<Case> cases;
    return cases;
}

struct Registrar {
    Registrar(const char* suite, const char* name, void (*fn)()) { registry().push_back({suite, name, fn}); }
};

struct RequireFailed : std::exception {};

inline int& failure_count() {
    static int n = 0;
    return n;
}

inline void report_failure(const char* file, int line, const std::string& what) {
    ++failure_count();
    std::fprintf(stderr, "    FAILED %s:%d: %s\n", file, line, what.c_str());
}

template <typename A, typename B>
std::string describe(const char* ea, const char* eb, const A& a, const B& b, const char* op) {
    std::ostringstream os;
    os << ea << ' ' << op << ' ' << eb << "  (" << a << " vs " << b << ")";
    return os.str();
}

}  // namespace lumen::testing

#define LUMEN_CONCAT_(a, b) a##b
#define LUMEN_CONCAT(a, b) LUMEN_CONCAT_(a, b)

#define TEST_CASE(suite, name)                                                                         \
    static void LUMEN_CONCAT(test_##suite##_, name)();                                                 \
    static const ::lumen::testing::Registrar LUMEN_CONCAT(reg_##suite##_, name)(                       \
        #suite, #name, &LUMEN_CONCAT(test_##suite##_, name));                                          \
    static void LUMEN_CONCAT(test_##suite##_, name)()

#define CHECK(expr)                                                                       \
    do {                                                                                  \
        if (!(expr)) ::lumen::testing::report_failure(__FILE__, __LINE__, #expr);         \
    } while (0)

#define REQUIRE(expr)                                                                     \
    do {                                                                                  \
        if (!(expr)) {                                                                    \
            ::lumen::testing::report_failure(__FILE__, __LINE__, #expr);                  \
            throw ::lumen::testing::RequireFailed{};                                      \
        }                                                                                 \
    } while (0)

#define CHECK_EQ(a, b)                                                                                    \
    do {                                                                                                  \
        const auto& lumen_a_ = (a);                                                                       \
        const auto& lumen_b_ = (b);                                                                       \
        if (!(lumen_a_ == lumen_b_))                                                                      \
            ::lumen::testing::report_failure(__FILE__, __LINE__,                                          \
                                             ::lumen::testing::describe(#a, #b, lumen_a_, lumen_b_, "=="));\
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                                             \
    do {                                                                                                  \
        const double lumen_a_ = static_cast<double>(a);                                                   \
        const double lumen_b_ = static_cast<double>(b);                                                   \
        if (!(std::fabs(lumen_a_ - lumen_b_) <= static_cast<double>(tol)))                                \
            ::lumen::testing::report_failure(__FILE__, __LINE__,                                          \
                                             ::lumen::testing::describe(#a, #b, lumen_a_, lumen_b_, "~="));\
    } while (0)

#define CHECK_THROWS(expr)                                                                \
    do {                                                                                  \
        bool lumen_threw_ = false;                                                        \
        try {                                                                             \
            (void)(expr);                                                                 \
        } catch (const std::exception&) {                                                 \
            lumen_threw_ = true;                                                          \
        }                                                                                 \
        if (!lumen_threw_) ::lumen::testing::report_failure(__FILE__, __LINE__, "no throw: " #expr); \
    } while (0)
