#include "crx_allocation_probe.h"

#include <cstdlib>
#include <limits>
#include <new>

#ifdef _WIN32
#include <malloc.h>
#endif

namespace {
thread_local std::size_t allocation_count = 0;

auto Allocate(std::size_t size) -> void * {
  ++allocation_count;
  if (void *memory = std::malloc(size == 0 ? 1 : size)) {
    return memory;
  }
  // Test-only replacement: an exhausted heap terminates this test process.
  std::abort();
}

auto AllocateAligned(std::size_t size, std::size_t alignment) -> void * {
  ++allocation_count;
  size = size == 0 ? 1 : size;
#ifdef _WIN32
  void *memory = _aligned_malloc(size, alignment);
#else
  if (size > std::numeric_limits<std::size_t>::max() - (alignment - 1)) {
    std::abort();
  }
  const std::size_t rounded_size =
      ((size + alignment - 1) / alignment) * alignment;
  void *memory = std::aligned_alloc(alignment, rounded_size);
#endif
  if (memory != nullptr) {
    return memory;
  }
  std::abort();
}

void FreeAligned(void *memory) noexcept {
#ifdef _WIN32
  _aligned_free(memory);
#else
  std::free(memory);
#endif
}
} // namespace

auto crx::test::AllocationCount() -> std::size_t { return allocation_count; }

auto operator new(std::size_t size) -> void * { return Allocate(size); }

auto operator new[](std::size_t size) -> void * { return Allocate(size); }

void operator delete(void *memory) noexcept { std::free(memory); }

void operator delete[](void *memory) noexcept { std::free(memory); }

void operator delete(void *memory, std::size_t) noexcept { std::free(memory); }

void operator delete[](void *memory, std::size_t) noexcept {
  std::free(memory);
}

auto operator new(std::size_t size, std::align_val_t alignment) -> void * {
  return AllocateAligned(size, static_cast<std::size_t>(alignment));
}

auto operator new[](std::size_t size, std::align_val_t alignment) -> void * {
  return AllocateAligned(size, static_cast<std::size_t>(alignment));
}

void operator delete(void *memory, std::align_val_t) noexcept {
  FreeAligned(memory);
}

void operator delete[](void *memory, std::align_val_t) noexcept {
  FreeAligned(memory);
}

void operator delete(void *memory, std::size_t, std::align_val_t) noexcept {
  FreeAligned(memory);
}

void operator delete[](void *memory, std::size_t, std::align_val_t) noexcept {
  FreeAligned(memory);
}
