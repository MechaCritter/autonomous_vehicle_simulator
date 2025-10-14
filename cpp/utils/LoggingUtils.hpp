#pragma once
#include <include/spdlog/spdlog.h>
#include <unordered_map>
#include <memory>
#include <string>
#include <include/spdlog/sinks/stdout_color_sinks.h>
#include <fstream>
#include <cstdlib>
#include "../include/nlohmann/json.hpp"

/**
 * @brief Issue a warning exactly once per call site (thread-safe).
 *
 * The message expression is evaluated at the call site (so it may reference
 * locals), then moved into the once-only lambda via init-capture.
 *
 * Usage:
 *   WARN_ONCE(std::string("foo ") + std::to_string(x));
 *   // or with fmt/spdlog:
 *   WARN_ONCE(fmt::format("value = {}", x));
 */
#define WARN_ONCE(MSG_EXPR)                                                     \
do {                                                                        \
static std::once_flag _warn_once_flag;                                  \
auto _warn_once_tmp_msg = (MSG_EXPR); /* evaluate in caller */          \
std::call_once(_warn_once_flag, [m = std::move(_warn_once_tmp_msg)]() { \
/* swap this line to spdlog::warn("{}", m); if you use spdlog */    \
std::cerr << "[Warning] " << m << '\n';                             \
});                                                                     \
} while (0)

namespace utils {

    class LoggingConfig {
    public:
        static LoggingConfig& instance() {
            static LoggingConfig inst;
            return inst;
        }
        spdlog::level::level_enum levelFor(const std::string& logger) const {
            auto it = table_.find(logger);
            return it != table_.end() ? it->second : spdlog::level::info;
        }
    private:
        LoggingConfig() {
            std::string path = std::getenv("VEHICLE_SIM_LOG_CFG") ?
                               std::getenv("VEHICLE_SIM_LOG_CFG") :
                               "/home/critter/workspace/autonomous_vehicle_simulator/cpp/res/logging_config.json";
            std::ifstream f(path);
            if (!f.good()) {
                spdlog::warn("Logging config '{}' not found – defaulting to INFO", path);
                return;
            }
            nlohmann::json j;  f >> j;
            for (auto& [key, val] : j.items())
                table_[key] = parseLevel(val.get<std::string>());
        }
        std::unordered_map<std::string, spdlog::level::level_enum> table_;
        static spdlog::level::level_enum parseLevel(std::string lvl) {
            std::ranges::transform(lvl, lvl.begin(), ::tolower);
            if (lvl == "trace")   return spdlog::level::trace;
            if (lvl == "debug")   return spdlog::level::debug;
            if (lvl == "info")    return spdlog::level::info;
            if (lvl == "warn")    return spdlog::level::warn;
            if (lvl == "error")   return spdlog::level::err;
            if (lvl == "critical")return spdlog::level::critical;
            return spdlog::level::info;          // fallback
        }
    };

    inline std::shared_ptr<spdlog::logger> getLogger(const std::string& name) {
        auto lg = spdlog::get(name);
        if (!lg) {
            lg = spdlog::stdout_color_mt(name);
            lg->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %n: %v");
        }
        lg->set_level(LoggingConfig::instance().levelFor(name));
        return lg;
    }

} // namespace utils
