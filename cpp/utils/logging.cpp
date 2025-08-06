#include "logging.h"
#include <fstream>
#include <cstdlib>
#include "../include/nlohmann/json.hpp"

using namespace utils;


static spdlog::level::level_enum parseLevel(std::string lvl)
{
    std::ranges::transform(lvl, lvl.begin(), ::tolower);
    if (lvl == "trace")   return spdlog::level::trace;
    if (lvl == "debug")   return spdlog::level::debug;
    if (lvl == "info")    return spdlog::level::info;
    if (lvl == "warn")    return spdlog::level::warn;
    if (lvl == "error")   return spdlog::level::err;
    if (lvl == "critical")return spdlog::level::critical;
    return spdlog::level::info;          // fallback
}

LoggingConfig::LoggingConfig()
{
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

LoggingConfig& LoggingConfig::instance()
{
    static LoggingConfig inst;
    return inst;
}

spdlog::level::level_enum
LoggingConfig::levelFor(const std::string& logger) const
{
    auto it = table_.find(logger);
    return it != table_.end() ? it->second : spdlog::level::info;
}

std::shared_ptr<spdlog::logger>
utils::getLogger(const std::string& name)
{
    auto lg = spdlog::get(name);
    if (!lg) {
        lg = spdlog::stdout_color_mt(name);
        lg->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %n: %v");
    }
    lg->set_level(LoggingConfig::instance().levelFor(name));
    return lg;
}
