#pragma once
#include <include/spdlog/spdlog.h>
#include <unordered_map>
#include <memory>
#include <string>
#include <include/spdlog/sinks/stdout_color_sinks.h>

namespace utils {

    /**
     * @brief Singleton that loads per-module log levels from a JSON config file.
     *
     * The file path is taken from the env-var `VEHICLE_SIM_LOG_CFG`
     * or defaults to `config/logging.json` relative to the binary.
     */
    class LoggingConfig {
    public:
        static LoggingConfig& instance();                       ///< lazy-initialised
        spdlog::level::level_enum levelFor(const std::string& logger) const;

    private:
        LoggingConfig();                                        ///< reads JSON once
        std::unordered_map<std::string, spdlog::level::level_enum> table_;
    };

    /**
     * @brief Fetch or create a logger with a level set from @ref LoggingConfig.
     * @param name  Logger name (e.g. `"lidar"`, `"Map2D"`).
     */
    std::shared_ptr<spdlog::logger> getLogger(const std::string& name);

} // namespace utils
