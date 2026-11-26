#include "lumen/scheduler.hpp"

#include <algorithm>
#include <atomic>
#include <deque>
#include <exception>
#include <mutex>
#include <optional>
#include <thread>

namespace lumen {
namespace {

class TaskQueue {
public:
    void push_back(std::size_t t) { items_.push_back(t); }

    std::optional<std::size_t> pop_front() {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (items_.empty()) {
            return std::nullopt;
        }
        const std::size_t t = items_.front();
        items_.pop_front();
        return t;
    }

    std::optional<std::size_t> steal_back() {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (items_.empty()) {
            return std::nullopt;
        }
        const std::size_t t = items_.back();
        items_.pop_back();
        return t;
    }

private:
    std::mutex mutex_;
    std::deque<std::size_t> items_;
};

}  // namespace

SchedulerStats run_work_stealing(std::size_t task_count, unsigned threads,
                                 const std::function<void(std::size_t, unsigned)>& task) {
    if (threads == 0) {
        threads = std::max(1U, std::thread::hardware_concurrency());
    }
    threads = static_cast<unsigned>(std::min<std::size_t>(threads, std::max<std::size_t>(task_count, 1)));

    SchedulerStats stats;
    stats.workers = threads;
    stats.tasks = task_count;
    stats.tasks_per_worker.assign(threads, 0);
    if (task_count == 0) {
        return stats;
    }

    std::vector<TaskQueue> queues(threads);
    for (std::size_t i = 0; i < task_count; ++i) {
        queues[i * threads / task_count].push_back(i);  // contiguous chunks keep neighbouring tiles together
    }

    std::atomic<bool> stop{false};
    std::atomic<std::size_t> steals{0};
    std::exception_ptr failure;
    std::mutex failure_mutex;

    auto worker = [&](unsigned self) {
        while (!stop.load(std::memory_order_relaxed)) {
            std::optional<std::size_t> job = queues[self].pop_front();
            if (!job) {
                for (unsigned k = 1; k < threads && !job; ++k) {
                    job = queues[(self + k) % threads].steal_back();
                }
                if (!job) {
                    return;  // no queue has work and tasks are never added after start
                }
                steals.fetch_add(1, std::memory_order_relaxed);
            }
            try {
                task(*job, self);
            } catch (...) {
                const std::lock_guard<std::mutex> lock(failure_mutex);
                if (!failure) {
                    failure = std::current_exception();
                }
                stop.store(true, std::memory_order_relaxed);
                return;
            }
            ++stats.tasks_per_worker[self];  // each slot is written by exactly one thread
        }
    };

    std::vector<std::thread> pool;
    pool.reserve(threads - 1);
    for (unsigned i = 1; i < threads; ++i) {
        pool.emplace_back(worker, i);
    }
    worker(0);
    for (auto& t : pool) {
        t.join();
    }
    stats.steals = steals.load();
    if (failure) {
        std::rethrow_exception(failure);
    }
    return stats;
}

}  // namespace lumen
