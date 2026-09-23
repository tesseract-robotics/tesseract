/**
 * @file logging.cpp
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

#include <tesseract/common/logging.h>

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace tesseract::common
{
namespace
{
struct RecordHandlerEntry
{
  LogRecordHandlerId id{ 0 };
  LogRecordHandler handler;
  std::mutex mutex;
  std::condition_variable condition;
  bool active{ true };
  std::size_t in_flight{ 0 };
  std::unordered_map<std::thread::id, std::size_t> callbacks_by_thread;
};

struct LoggingState
{
  std::mutex mutex;
  LogRecordHandlerId next_handler_id{ 1 };
  std::vector<std::shared_ptr<RecordHandlerEntry>> handlers;
};

LoggingState& state()
{
  // The logging state intentionally lives until process exit so logging remains available during static teardown.
  // NOLINTNEXTLINE(cppcoreguidelines-owning-memory,cppcoreguidelines-avoid-non-const-global-variables)
  static auto* logging_state = new LoggingState();
  return *logging_state;
}

std::string formatAttribute(const LogAttribute& attribute)
{
  return std::visit(
      [](const auto& value) {
        using ValueType = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<ValueType, bool>)
          return value ? std::string("true") : std::string("false");
        else if constexpr (std::is_same_v<ValueType, std::string>)
          return value;
        else
          return std::to_string(value);
      },
      attribute);
}

std::string formatAttributes(const LogAttributes& attributes)
{
  std::vector<std::pair<std::string, std::string>> values;
  values.reserve(attributes.size());
  for (const auto& attribute : attributes)
    values.emplace_back(attribute.first, formatAttribute(attribute.second));

  std::sort(values.begin(), values.end());
  std::ostringstream stream;
  for (const auto& attribute : values)
    stream << ' ' << attribute.first << '=' << attribute.second;
  return stream.str();
}
}  // namespace

std::shared_ptr<spdlog::logger> getLogger(std::string_view name)
{
  const std::string logger_name(name);
  if (auto logger = spdlog::get(logger_name))
    return logger;

  std::shared_ptr<spdlog::logger> base_logger;
  if (logger_name != "tesseract")
    base_logger = getLogger();

  auto& logging_state = state();
  std::lock_guard<std::mutex> lock(logging_state.mutex);
  if (auto logger = spdlog::get(logger_name))
    return logger;

  std::shared_ptr<spdlog::logger> logger;
  if (logger_name == "tesseract")
  {
    auto sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    logger = std::make_shared<spdlog::logger>(logger_name, std::move(sink));
    logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    logger->set_level(spdlog::level::info);
  }
  else
  {
    logger = base_logger->clone(logger_name);
  }

  spdlog::register_logger(logger);
  return logger;
}

LogRecordHandlerId addLogRecordHandler(LogRecordHandler handler)
{
  if (!handler)
    return 0;

  auto& logging_state = state();
  std::lock_guard<std::mutex> lock(logging_state.mutex);
  const auto id = logging_state.next_handler_id++;
  auto entry = std::make_shared<RecordHandlerEntry>();
  entry->id = id;
  entry->handler = std::move(handler);
  logging_state.handlers.push_back(std::move(entry));
  return id;
}

bool removeLogRecordHandler(LogRecordHandlerId id) noexcept
{
  if (id == 0)
    return false;

  auto& logging_state = state();
  std::shared_ptr<RecordHandlerEntry> handler_entry;
  {
    std::lock_guard<std::mutex> lock(logging_state.mutex);
    const auto entry = std::find_if(logging_state.handlers.begin(),
                                    logging_state.handlers.end(),
                                    [id](const auto& item) { return item->id == id; });
    if (entry == logging_state.handlers.end())
      return false;
    handler_entry = *entry;
  }

  std::unique_lock<std::mutex> handler_lock(handler_entry->mutex);
  if (!handler_entry->active)
    return false;

  handler_entry->active = false;
  const auto current_thread = std::this_thread::get_id();
  const auto current_callbacks = handler_entry->callbacks_by_thread.find(current_thread);
  const auto current_callback_count =
      current_callbacks == handler_entry->callbacks_by_thread.end() ? 0 : current_callbacks->second;
  handler_entry->condition.wait(handler_lock, [&handler_entry, current_callback_count] {
    return handler_entry->in_flight == current_callback_count;
  });
  handler_lock.unlock();

  std::lock_guard<std::mutex> lock(logging_state.mutex);
  const auto entry = std::find_if(logging_state.handlers.begin(),
                                  logging_state.handlers.end(),
                                  [&handler_entry](const auto& item) { return item == handler_entry; });
  if (entry != logging_state.handlers.end())
    logging_state.handlers.erase(entry);
  return true;
}

void emitLogRecord(const LogRecord& record) noexcept
{
  try
  {
    auto logger = getLogger(record.logger_name);
    if (!logger->should_log(record.level))
      return;

    logger->log(record.source_location, record.level, "{}{}", record.message, formatAttributes(record.attributes));

    std::vector<std::shared_ptr<RecordHandlerEntry>> handlers;
    auto& logging_state = state();
    {
      std::lock_guard<std::mutex> lock(logging_state.mutex);
      handlers.reserve(logging_state.handlers.size());
      for (const auto& entry : logging_state.handlers)
        handlers.push_back(entry);
    }

    for (const auto& entry : handlers)
    {
      {
        std::lock_guard<std::mutex> lock(entry->mutex);
        if (!entry->active)
          continue;

        ++entry->in_flight;
        try
        {
          ++entry->callbacks_by_thread[std::this_thread::get_id()];
        }
        catch (...)
        {
          --entry->in_flight;
          entry->condition.notify_all();
          throw;
        }
      }

      bool handler_failed{ false };
      try
      {
        entry->handler(record);
      }
      catch (...)
      {
        handler_failed = true;
      }

      {
        std::lock_guard<std::mutex> lock(entry->mutex);
        --entry->in_flight;
        const auto callback_thread = entry->callbacks_by_thread.find(std::this_thread::get_id());
        if (--callback_thread->second == 0)
          entry->callbacks_by_thread.erase(callback_thread);
        entry->condition.notify_all();
      }

      if (handler_failed)
        continue;
    }
  }
  catch (...)
  {
    return;
  }
}

}  // namespace tesseract::common