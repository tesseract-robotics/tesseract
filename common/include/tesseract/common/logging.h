/**
 * @file logging.h
 * @brief Structured logging extensions for spdlog.
 *
 * @author Levi Armstrong
 * @date September 24, 2026
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_COMMON_LOGGING_H
#define TESSERACT_COMMON_LOGGING_H

#include <spdlog/fmt/fmt.h>
#include <spdlog/logger.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>

namespace tesseract::common
{
/** @brief Scalar value supported by a structured log-record attribute. */
using LogAttribute = std::variant<bool, std::int64_t, std::uint64_t, double, std::string>;

/** @brief Collection of structured attributes indexed by attribute name. */
using LogAttributes = std::unordered_map<std::string, LogAttribute>;

/** @brief Structured representation of a Tesseract log event. */
struct LogRecord
{
  /** @brief Time at which the record was created. */
  std::chrono::system_clock::time_point timestamp{ std::chrono::system_clock::now() };

  /** @brief Native spdlog severity of the event. */
  spdlog::level::level_enum level{ spdlog::level::info };

  /** @brief Name of the spdlog logger that owns the event. */
  std::string logger_name{ "tesseract" };

  /** @brief Logical component that produced the event, if known. */
  std::string component_name;

  /** @brief Fully formatted human-readable event message. */
  std::string message;

  /** @brief Source file, line, and function at which the event originated. */
  spdlog::source_loc source_location;

  /** @brief Identifier of the thread that created the record. */
  std::thread::id thread_id{ std::this_thread::get_id() };

  /** @brief Typed application-defined attributes attached to the event. */
  LogAttributes attributes;
};

/** @brief Callback invoked for each structured record that passes spdlog filtering. */
using LogRecordHandler = std::function<void(const LogRecord&)>;

/** @brief Opaque identifier assigned to a registered structured-record handler. */
using LogRecordHandlerId = std::uint64_t;

/**
 * @brief Get or create a registered spdlog logger.
 *
 * The default `tesseract` logger writes colorized messages to standard error. A newly created named logger clones the
 * current sinks, formatter, level, error handler, and flush level of the default logger.
 *
 * @param name Logger name. An empty name is permitted and follows normal spdlog registry semantics.
 * @return Shared ownership of the registered logger.
 */
std::shared_ptr<spdlog::logger> getLogger(std::string_view name = "tesseract");

/**
 * @brief Register a callback that receives structured log records.
 *
 * Handlers are invoked synchronously after the record has been written to its spdlog sinks. Exceptions thrown by a
 * handler are isolated and do not escape the logging call or prevent later handlers from running.
 *
 * @param handler Callback to register.
 * @return A nonzero identifier for a valid callback, or zero when @p handler is empty.
 */
LogRecordHandlerId addLogRecordHandler(LogRecordHandler handler);

/**
 * @brief Unregister a structured-record callback.
 *
 * Removal prevents a handler that has not yet started from being called by an in-flight dispatch and waits for
 * callbacks already running on other threads to return. When called by the handler itself, removal returns without
 * waiting for the current callback. State captured by a handler must remain valid until any callback that has already
 * started has returned.
 *
 * @param id Identifier returned by addLogRecordHandler().
 * @return True when a registered handler was removed; otherwise false.
 */
bool removeLogRecordHandler(LogRecordHandlerId id) noexcept;

/**
 * @brief Emit a structured record through its spdlog logger and all registered handlers.
 *
 * Records rejected by the selected logger's native spdlog level are discarded before sink or handler dispatch.
 * Structured attributes are appended to the spdlog message in deterministic key order. Logging and handler failures
 * are contained and do not escape this function.
 *
 * @param record Record to emit.
 */
void emitLogRecord(const LogRecord& record) noexcept;

namespace detail
{
/**
 * @brief Format and emit a record for the Tesseract convenience macros.
 *
 * Native spdlog filtering is checked before formatting. A formatting exception is converted to a fallback message so
 * logging remains nonthrowing.
 *
 * @tparam Args Types of the format arguments.
 * @param logger_name Name of the destination logger.
 * @param level Native spdlog severity.
 * @param source_location Source location captured by the calling macro.
 * @param format Compile-time checked fmt format string.
 * @param args Values referenced by @p format.
 */
template <typename... Args>
void log(std::string_view logger_name,
         spdlog::level::level_enum level,
         spdlog::source_loc source_location,
         fmt::format_string<Args...> format,
         Args&&... args) noexcept
{
  auto logger = getLogger(logger_name);
  if (!logger->should_log(level))
    return;

  LogRecord record;
  record.level = level;
  record.logger_name = logger->name();
  record.source_location = source_location;
  try
  {
    record.message = fmt::format(format, std::forward<Args>(args)...);
  }
  catch (...)
  {
    record.message = "Unable to format log message";
  }
  emitLogRecord(record);
}
}  // namespace detail

}  // namespace tesseract::common

/** @brief Capture the current source file, line, and function for a log record. */
#define TESSERACT_LOG_SOURCE_LOCATION                                                                                  \
  spdlog::source_loc { __FILE__, __LINE__, SPDLOG_FUNCTION }

/**
 * @brief Emit a formatted structured record through a named logger.
 * @param name Logger name or expression convertible to `std::string_view`.
 * @param severity A native spdlog level token such as `info`, `warn`, or `err`.
 * @param ... An fmt-compatible format string followed by its arguments.
 */
#define TESSERACT_LOG_NAMED(name, severity, ...)                                                                       \
  ::tesseract::common::detail::log(name, spdlog::level::severity, TESSERACT_LOG_SOURCE_LOCATION, __VA_ARGS__)

/** @brief Emit a trace record through the default `tesseract` logger. */
#define TESSERACT_LOG_TRACE(...) TESSERACT_LOG_NAMED("tesseract", trace, __VA_ARGS__)
/** @brief Emit a debug record through the default `tesseract` logger. */
#define TESSERACT_LOG_DEBUG(...) TESSERACT_LOG_NAMED("tesseract", debug, __VA_ARGS__)
/** @brief Emit an informational record through the default `tesseract` logger. */
#define TESSERACT_LOG_INFO(...) TESSERACT_LOG_NAMED("tesseract", info, __VA_ARGS__)
/** @brief Emit a warning record through the default `tesseract` logger. */
#define TESSERACT_LOG_WARN(...) TESSERACT_LOG_NAMED("tesseract", warn, __VA_ARGS__)
/** @brief Emit an error record through the default `tesseract` logger. */
#define TESSERACT_LOG_ERROR(...) TESSERACT_LOG_NAMED("tesseract", err, __VA_ARGS__)
/** @brief Emit a critical record through the default `tesseract` logger. */
#define TESSERACT_LOG_CRITICAL(...) TESSERACT_LOG_NAMED("tesseract", critical, __VA_ARGS__)

/** @brief Emit a trace record through a named logger. */
#define TESSERACT_LOG_TRACE_NAMED(name, ...) TESSERACT_LOG_NAMED(name, trace, __VA_ARGS__)
/** @brief Emit a debug record through a named logger. */
#define TESSERACT_LOG_DEBUG_NAMED(name, ...) TESSERACT_LOG_NAMED(name, debug, __VA_ARGS__)
/** @brief Emit an informational record through a named logger. */
#define TESSERACT_LOG_INFO_NAMED(name, ...) TESSERACT_LOG_NAMED(name, info, __VA_ARGS__)
/** @brief Emit a warning record through a named logger. */
#define TESSERACT_LOG_WARN_NAMED(name, ...) TESSERACT_LOG_NAMED(name, warn, __VA_ARGS__)
/** @brief Emit an error record through a named logger. */
#define TESSERACT_LOG_ERROR_NAMED(name, ...) TESSERACT_LOG_NAMED(name, err, __VA_ARGS__)
/** @brief Emit a critical record through a named logger. */
#define TESSERACT_LOG_CRITICAL_NAMED(name, ...) TESSERACT_LOG_NAMED(name, critical, __VA_ARGS__)

#endif  // TESSERACT_COMMON_LOGGING_H