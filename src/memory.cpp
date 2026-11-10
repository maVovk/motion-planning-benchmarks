#include "mpb/memory.hpp"

#include <cstdint>
#include <fstream>
#include <stdexcept>

#if defined(__linux__)
#include <malloc.h>
#include <unistd.h>
#elif defined(__APPLE__)
#include <mach/mach.h>
#include <malloc/malloc.h>
#else
#error "Memory snapshots require Linux or macOS"
#endif


namespace mpb {
MemorySnapshot memorySnapshot() {
#if defined(__linux__)
  // Open the file before sampling heap to avoid including stream setup in the delta.
  std::ifstream status("/proc/self/statm");
  std::int64_t total_pages = 0, resident_pages = 0;
  if (!(status >> total_pages >> resident_pages)) throw std::runtime_error("Cannot read statm");
  const auto page_size = sysconf(_SC_PAGESIZE);
  if (page_size <= 0) throw std::runtime_error("Cannot read page size");
  const auto heap = mallinfo2();
  return {static_cast<std::int64_t>(heap.uordblks) + static_cast<std::int64_t>(heap.hblkhd),
          resident_pages * page_size};
#else
  malloc_statistics_t heap{};
  malloc_zone_statistics(nullptr, &heap);
  mach_task_basic_info_data_t info{};
  mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
  if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
                reinterpret_cast<task_info_t>(&info), &count) != KERN_SUCCESS) {
    throw std::runtime_error("task_info failed");
  }
  return {static_cast<std::int64_t>(heap.size_in_use), static_cast<std::int64_t>(info.resident_size)};
#endif
}


}  // namespace mpb
