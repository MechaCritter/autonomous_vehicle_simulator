# Documentation

## utils/logging — Developer README

### Purpose
Provide a **central, reusable** way to create `spdlog` loggers whose level is controlled at runtime from a JSON file—no 
more hard-coded `info` verbosity.

### Key API (surface)

| Function | Location | Description |
|----------|-----------|-------------|
| `std::shared_ptr<spdlog::logger> utils::getLogger(const std::string& name)` | `utils/logging.h` | Fetch (or lazily create) a named logger and automatically set its level/pattern. |
| `LoggingConfig::instance()` | *internal* | Singleton that parses the JSON exactly once. |
| `LoggingConfig::levelFor(name)` | *internal* | Module-level lookup. |

### Configuration

```json
// config/logging.json
{
  "root":  "info",
  "lidar": "debug",
  "Map2D": "warn",
  "Vehicle": "trace"
}
```

* **Location:** default `config/logging.json`, or override with  
  `export VEHICLE_SIM_LOG_CFG=/path/to/alt.json`.
* **Keys** are logger names; values are one of  
  `trace | debug | info | warn | error | critical`.

Unlisted loggers inherit `root` (fallback = `info`).

### Usage

```cpp
#include "../utils/logging.h"

class Lidar2D {
    [[nodiscard]] spdlog::logger& logger() const {
        static auto lg = utils::getLogger("lidar");
        return *lg;
    }
};
```

### Build Notes
* **Source files:** `utils/logging.{h,cpp}`  
  Ensure `logging.cpp` is compiled into your `utils` library:
  ```cmake
  add_library(utils utils.cpp logging.cpp)
  ```
* Requires **nlohmann/json.hpp** (already vendored under `include/`).


---