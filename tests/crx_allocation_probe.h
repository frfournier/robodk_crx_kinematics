#pragma once

#include <cstddef>

// Test executable only: counts replaceable scalar/array/aligned C++ allocations
// on the calling thread. Direct CRT allocations are not intercepted here.
namespace crx::test {
auto AllocationCount() -> std::size_t;
} // namespace crx::test
