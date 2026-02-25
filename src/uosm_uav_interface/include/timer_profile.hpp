#pragma once

#include <chrono>
#include <string>
#include <map>

namespace uosm
{
    namespace util
    {
        /**
         * @brief Class to measure the time duration of a code block using steady_clock.
         */
        class TimerProfiler
        {
        public:
            using Clock = std::chrono::steady_clock;
            using TimePoint = Clock::time_point;
            using Duration = std::chrono::nanoseconds;

            TimerProfiler(const std::string &section_name,
                          std::map<std::string, Duration> &durations)
                : section_name_(section_name),
                  durations_(durations),
                  start_time_(Clock::now()) {}

            ~TimerProfiler()
            {
                auto dur = std::chrono::duration_cast<Duration>(Clock::now() - start_time_);
                durations_.insert_or_assign(section_name_, dur);
            }

        private:
            std::string section_name_;
            std::map<std::string, Duration> &durations_;
            TimePoint start_time_;
        };

        inline std::string formatDuration(const std::chrono::nanoseconds &dur)
        {
            auto ns = dur.count();
            if (ns > 1'000'000)
            {
                return std::to_string(ns / 1'000'000.0) + " ms";
            }
            else if (ns > 1'000)
            {
                return std::to_string(ns / 1'000.0) + " µs";
            }
            else
            {
                return std::to_string(ns) + " ns";
            }
        }
    } // namespace util
} // namespace uosm

#define PROFILE_SECTION(name) \
    uosm::util::TimerProfiler timer_##__LINE__ { name, section_durations_ }
