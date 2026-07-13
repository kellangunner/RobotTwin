// Minimal self-registering test harness — no external dependency, keeps the
// core buildable offline on every target platform.
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace rtest {

struct Registry {
  static Registry& instance() {
    static Registry r;
    return r;
  }
  std::vector<std::pair<std::string, std::function<void()>>> tests;
  std::string current;
  int checks = 0;
  int failures = 0;
};

struct Registrar {
  Registrar(std::string name, std::function<void()> fn) {
    Registry::instance().tests.emplace_back(std::move(name), std::move(fn));
  }
};

inline void fail(const char* file, int line, const std::string& msg) {
  auto& r = Registry::instance();
  ++r.failures;
  std::printf("FAIL [%s] %s:%d\n      %s\n", r.current.c_str(), file, line, msg.c_str());
}

inline int runAll() {
  auto& r = Registry::instance();
  for (auto& [name, fn] : r.tests) {
    r.current = name;
    fn();
  }
  std::printf("\n%zu tests, %d checks, %d failures\n", r.tests.size(), r.checks, r.failures);
  return r.failures == 0 ? 0 : 1;
}

} // namespace rtest

#define RT_TEST(name)                                                    \
  static void rt_test_##name();                                          \
  static ::rtest::Registrar rt_reg_##name(#name, rt_test_##name);        \
  static void rt_test_##name()

#define CHECK(cond)                                                      \
  do {                                                                   \
    ++::rtest::Registry::instance().checks;                              \
    if (!(cond)) ::rtest::fail(__FILE__, __LINE__, "CHECK: " #cond);     \
  } while (0)

#define CHECK_CLOSE(a, b, tol)                                           \
  do {                                                                   \
    ++::rtest::Registry::instance().checks;                              \
    const double rt_a = (a), rt_b = (b);                                 \
    if (!(std::abs(rt_a - rt_b) <= (tol)))                               \
      ::rtest::fail(__FILE__, __LINE__,                                  \
                    std::string("CHECK_CLOSE: " #a " = ") +              \
                        std::to_string(rt_a) + " vs " #b " = " +         \
                        std::to_string(rt_b));                           \
  } while (0)
