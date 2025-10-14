#pragma once
#include <chrono>
#include <string>
#include <string_view>
#include <unordered_map>
#include <cstdint>
#include <algorithm>
#include <mutex>

/**
 * @brief Aggregate timing for a named section across frames.
 */
struct ProfSec {
    uint64_t calls = 0;
    std::chrono::nanoseconds total{0};
    std::chrono::nanoseconds max{0};

    void reset() {
        calls = 0;
        total = std::chrono::nanoseconds{0};
        max = std::chrono::nanoseconds{0};
    }
};

/**
 * @brief Frame-scoped profiler that aggregates section timings and logs every N frames.
 * @note Not thread-safe; use in the single simulation thread only.
 */
class FrameProfiler {
public:
    /**
     * @brief Construct a FrameProfiler.
     * @param log_every_frames Log every N frames (default 100).
     */
    explicit FrameProfiler(std::string name, uint32_t log_every_frames = 200) noexcept
        : name_{std::move(name)}, log_every_(log_every_frames) {}

    /**
     * @brief Start a new frame. Resets per-frame totals. If
     * number of frames is multiple of log_every_frames, print
      * a summary.
     */
    void begin_frame() noexcept {
        ++frame_index_;
        frame_start_ = clock::now();
    }

    /**
     * @brief End a frame. Optionally record total frame time (excluding sleep if you measure sleep separately).
     */
    void end_frame() noexcept {
        const auto frame_end = clock::now();
        total_work_accumulated_ += std::chrono::duration_cast<std::chrono::nanoseconds>(frame_end - frame_start_);
        if (frame_index_ % log_every_ == 0) {
            log_and_reset();
        }
    }

    /**
     * @brief Add measured nanoseconds to a section.
     * @param name Section name.
     * @param ns Time in nanoseconds to add.
     */
    void add(std::string_view name, std::chrono::nanoseconds ns) noexcept {
        auto [it, inserted] = sections_.try_emplace(std::string{name}, ProfSec{});
        auto& s = it->second;
        ++s.calls;
        s.total += ns;
        s.max = std::max(s.max, ns);
    }

    /**
     * @brief Build a human-readable summary string (percentages and absolute times). then,
     * reset all counters.
     *
     * @param tag Optional tag for log output.
     */
    void log_and_reset() {
        // Compute total work time across all sections except Sleep if present.
        const double total_work_ms = total_work_accumulated_.count() / 1e6;

        std::cout << "[" << name_ << "] Frames: " << frame_index_ << "\n";
        std::cout << "[" << name_ << "] Total work accumulated: " << std::fixed << std::setprecision(3)
                  << total_work_ms << " ms.\n";

        // Sort sections by total time descending
        std::vector<std::pair<std::string, ProfSec>> rows(sections_.begin(), sections_.end());
        std::sort(rows.begin(), rows.end(),
                  [](auto& a, auto& b){ return a.second.total > b.second.total; });

        std::cout << "[" << name_ << "] "
                  << std::left << std::setw(28) << "Section"
                  << std::right << std::setw(8) << "calls"
                  << std::setw(12) << "avg(ms)"
                  << std::setw(12) << "max(ms)"
                  << std::setw(12) << "total(ms)"
                  << std::setw(8) << "%" << "\n";
        for (auto& [name, s] : rows) {
            const double tot_ms = s.total.count() / 1e6;
            const double avg_ms = (s.calls ? tot_ms / static_cast<double>(s.calls) : 0.0);
            const double percentage = (total_work_ms > 0 ? (tot_ms * 100.0) / total_work_ms : 0.0);
            std::cout << "[" << name_ << "] "
                      << std::left << std::setw(28) << name
                      << std::right << std::setw(8) << s.calls
                      << std::setw(12) << std::fixed << std::setprecision(3) << avg_ms
                        << std::setw(12) << s.max.count() / 1e6
                      << std::setw(12) << std::fixed << std::setprecision(3) << tot_ms
                      << std::setw(7) << std::setprecision(1) << percentage << "%" << "\n";
        }
        for (auto& kv : sections_) {
            kv.second.reset();
        }
        std::cout << std::flush;
        frame_index_ = 0; // reset counter after logging
        total_work_accumulated_ = std::chrono::nanoseconds{0};
    }

private:
    using clock = std::chrono::steady_clock;

    uint32_t log_every_;
    uint64_t frame_index_ = 0;
    std::string name_;

    std::chrono::time_point<clock> frame_start_;
    std::chrono::nanoseconds total_work_accumulated_{0};

    std::unordered_map<std::string, ProfSec> sections_;
};

/**
 * @brief RAII scope timer for a named section.
 */
class ScopedSection {
public:
    using clock = std::chrono::steady_clock;
    /**
     * @brief Construct a ScopedSection.
     * @param p Reference to FrameProfiler.
     * @param name Section name.
     */
    ScopedSection(FrameProfiler& p, std::string_view name) noexcept
        : prof_(p), name_(name), start_(clock::now()) {}
    /**
     * @brief Destructor. Adds elapsed time to the profiler section.
     */
    ~ScopedSection() noexcept {
        const auto end = clock::now();
        prof_.add(name_, std::chrono::duration_cast<std::chrono::nanoseconds>(end - start_));
    }
    ScopedSection(const ScopedSection&) = delete;
    ScopedSection& operator=(const ScopedSection&) = delete;

private:
    FrameProfiler& prof_;
    std::string_view name_;
    clock::time_point start_;
};
