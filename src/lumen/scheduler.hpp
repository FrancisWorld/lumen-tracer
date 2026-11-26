// Work-stealing task scheduler. Tasks (image tiles) are dealt out in
// contiguous chunks, one deque per worker. A worker pops from the front of
// its own deque for locality and, when empty, steals from the back of another
// worker's deque so that uneven tiles (glass, caustics) never leave cores idle.
#pragma once

#include <cstddef>
#include <functional>
#include <vector>

namespace lumen {

struct SchedulerStats {
    unsigned workers = 0;
    std::size_t tasks = 0;
    std::size_t steals = 0;
    std::vector<std::size_t> tasks_per_worker;
};

/// Runs `task(index, worker)` exactly once for every index in [0, task_count).
/// `threads == 0` uses the hardware concurrency. The first exception thrown by
/// a task stops the remaining work and is rethrown on the calling thread.
SchedulerStats run_work_stealing(std::size_t task_count, unsigned threads,
                                 const std::function<void(std::size_t task, unsigned worker)>& task);

}  // namespace lumen
