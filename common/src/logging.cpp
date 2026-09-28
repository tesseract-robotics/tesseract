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
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
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
  std::shared_ptr<spdlog::logger> default_logger;
  std::atomic<spdlog::logger*> default_logger_raw{ nullptr };
  LogRecordHandlerId next_handler_id{ 1 };
  std::atomic<std::size_t> active_handler_count{ 0 };
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

std::string formatMessage(const LogRecord& record)
{
  if (record.attributes.empty())
    return record.message;

  std::vector<std::pair<std::string, std::string>> values;
  values.reserve(record.attributes.size());
  std::size_t size = record.message.size();
  for (const auto& attribute : record.attributes)
  {
    values.emplace_back(attribute.first, formatAttribute(attribute.second));
    size += attribute.first.size() + values.back().second.size() + 2;
  }

  std::sort(values.begin(), values.end());
  std::string message;
  message.reserve(size);
  message.append(record.message);
  for (const auto& attribute : values)
  {
    message.push_back(' ');
    message.append(attribute.first);
    message.push_back('=');
    message.append(attribute.second);
  }
  return message;
}

std::shared_ptr<spdlog::logger> createDefaultLogger()
{
  auto sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
  auto logger = std::make_shared<spdlog::logger>("tesseract", std::move(sink));
  logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
  logger->set_level(spdlog::level::info);
  return logger;
}

void dispatchHandlers(const LogRecord& record)
{
  auto& logging_state = state();
  if (logging_state.active_handler_count.load(std::memory_order_acquire) == 0)
    return;

  std::vector<std::shared_ptr<RecordHandlerEntry>> handlers;
  {
    std::lock_guard<std::mutex> lock(logging_state.mutex);
    handlers = logging_state.handlers;
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
}  // namespace

std::shared_ptr<spdlog::logger> getLogger(std::string_view name)
{
  const std::string logger_name(name);
  auto& logging_state = state();
  if (logger_name == "tesseract")
  {
    std::lock_guard<std::mutex> lock(logging_state.mutex);
    if (logging_state.default_logger == nullptr)
    {
      logging_state.default_logger = spdlog::get(logger_name);
      if (logging_state.default_logger == nullptr)
      {
        logging_state.default_logger = createDefaultLogger();
        spdlog::register_logger(logging_state.default_logger);
      }
      logging_state.default_logger_raw.store(logging_state.default_logger.get(), std::memory_order_release);
    }
    return logging_state.default_logger;
  }

  if (auto logger = spdlog::get(logger_name))
    return logger;

  auto base_logger = getLogger();

  std::lock_guard<std::mutex> lock(logging_state.mutex);
  if (auto logger = spdlog::get(logger_name))
    return logger;

  auto logger = base_logger->clone(logger_name);
  spdlog::register_logger(logger);
  return logger;
}

void setLogger(std::shared_ptr<spdlog::logger> logger)
{
  if (logger == nullptr || logger->name() != "tesseract")
    throw std::invalid_argument("The default Tesseract logger must be non-null and named 'tesseract'");

  auto& logging_state = state();
  std::lock_guard<std::mutex> lock(logging_state.mutex);
  spdlog::drop("tesseract");
  spdlog::register_logger(logger);
  logging_state.default_logger = std::move(logger);
  logging_state.default_logger_raw.store(logging_state.default_logger.get(), std::memory_order_release);
}

bool isLogLevelEnabled(spdlog::level::level_enum level) noexcept
{
  auto* logger = detail::getDefaultLogger();
  return logger != nullptr && logger->should_log(level);
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
  logging_state.active_handler_count.fetch_add(1, std::memory_order_release);
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
  {
    logging_state.handlers.erase(entry);
    logging_state.active_handler_count.fetch_sub(1, std::memory_order_release);
  }
  return true;
}

namespace detail
{
spdlog::logger* getDefaultLogger() noexcept
{
  auto& logging_state = state();
  auto* logger = logging_state.default_logger_raw.load(std::memory_order_acquire);
  if (logger != nullptr)
    return logger;

  try
  {
    return getLogger().get();
  }
  catch (...)
  {
    return nullptr;
  }
}

void emitLogRecord(spdlog::logger& logger, const LogRecord& record) noexcept
{
  try
  {
    if (record.attributes.empty())
    {
      logger.log(
          record.source_location, record.level, spdlog::string_view_t(record.message.data(), record.message.size()));
    }
    else
    {
      const std::string message = formatMessage(record);
      logger.log(record.source_location, record.level, spdlog::string_view_t(message.data(), message.size()));
    }
    dispatchHandlers(record);
  }
  catch (...)
  {
    return;
  }
}
}  // namespace detail

void emitLogRecord(const LogRecord& record) noexcept
{
  try
  {
    auto logger = getLogger(record.logger_name);
    if (!logger->should_log(record.level))
      return;
    detail::emitLogRecord(*logger, record);
  }
  catch (...)
  {
    return;
  }
}

}  // namespace tesseract::common