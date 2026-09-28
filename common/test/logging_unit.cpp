#include <tesseract/common/logging.h>

#include <gtest/gtest.h>
#include <spdlog/sinks/null_sink.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace
{
class DefaultLoggerGuard
{
public:
  DefaultLoggerGuard() : logger_(tesseract::common::getLogger()->clone("tesseract")) {}

  ~DefaultLoggerGuard() { tesseract::common::setLogger(std::move(logger_)); }

  DefaultLoggerGuard(const DefaultLoggerGuard&) = delete;
  DefaultLoggerGuard& operator=(const DefaultLoggerGuard&) = delete;
  DefaultLoggerGuard(DefaultLoggerGuard&&) = delete;
  DefaultLoggerGuard& operator=(DefaultLoggerGuard&&) = delete;

private:
  std::shared_ptr<spdlog::logger> logger_;
};
}  // namespace

TEST(TesseractLoggingUnit, ReplacesDefaultLoggerAndChecksLevelWithoutRegistryLookup)
{
  DefaultLoggerGuard guard;
  auto stream = std::make_shared<std::ostringstream>();
  auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(*stream);
  sink->set_pattern("%n|%l|%v");
  auto logger = std::make_shared<spdlog::logger>("tesseract", sink);
  logger->set_level(spdlog::level::warn);
  tesseract::common::setLogger(logger);

  EXPECT_EQ(tesseract::common::getLogger(), logger);
  EXPECT_FALSE(tesseract::common::isLogLevelEnabled(spdlog::level::debug));
  EXPECT_TRUE(tesseract::common::isLogLevelEnabled(spdlog::level::warn));

  TESSERACT_LOG_INFO("filtered");
  TESSERACT_LOG_WARN("retained");
  logger->flush();

  EXPECT_EQ(stream->str().find("filtered"), std::string::npos);
  EXPECT_NE(stream->str().find("tesseract|warning|retained"), std::string::npos);
}

TEST(TesseractLoggingUnit, SupportsDirectDefaultLoggerConfiguration)
{
  DefaultLoggerGuard guard;
  auto logger = tesseract::common::getLogger();
  auto stream = std::make_shared<std::ostringstream>();
  logger->sinks() = { std::make_shared<spdlog::sinks::ostream_sink_mt>(*stream) };
  logger->set_level(spdlog::level::err);

  TESSERACT_LOG_WARN("filtered");
  TESSERACT_LOG_ERROR("retained");
  logger->flush();

  EXPECT_EQ(stream->str().find("filtered"), std::string::npos);
  EXPECT_NE(stream->str().find("retained"), std::string::npos);
}

TEST(TesseractLoggingUnit, RetainsOwnedDefaultLoggerAfterRegistryDrop)
{
  DefaultLoggerGuard guard;
  auto logger = tesseract::common::getLogger();
  spdlog::drop("tesseract");

  EXPECT_EQ(spdlog::get("tesseract"), nullptr);
  EXPECT_EQ(tesseract::common::getLogger(), logger);
}

TEST(TesseractLoggingUnit, RejectsInvalidDefaultLogger)
{
  EXPECT_THROW(tesseract::common::setLogger(nullptr), std::invalid_argument);
  EXPECT_THROW(tesseract::common::setLogger(std::make_shared<spdlog::logger>("other")), std::invalid_argument);
}

TEST(TesseractLoggingUnit, NamedLoggerClonesReplacementDefaultLogger)
{
  DefaultLoggerGuard guard;
  auto stream = std::make_shared<std::ostringstream>();
  auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(*stream);
  sink->set_pattern("%n|%l|%v");
  auto logger = std::make_shared<spdlog::logger>("tesseract", sink);
  logger->set_level(spdlog::level::debug);
  tesseract::common::setLogger(logger);

  auto named_logger = tesseract::common::getLogger("test.replacement_clone");
  TESSERACT_LOG_DEBUG_NAMED("test.replacement_clone", "cloned");
  named_logger->flush();

  EXPECT_EQ(named_logger->level(), spdlog::level::debug);
  EXPECT_NE(stream->str().find("test.replacement_clone|debug|cloned"), std::string::npos);
}

TEST(TesseractLoggingUnit, UsesNativeSpdlogLoggerAndSink)
{
  auto logger = tesseract::common::getLogger("test.native");
  auto stream = std::make_shared<std::ostringstream>();
  auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(*stream);
  sink->set_pattern("%n|%l|%v");
  logger->sinks() = { sink };
  logger->set_level(spdlog::level::trace);

  const std::string text = "percent % and braces {}";
  TESSERACT_LOG_INFO_NAMED("test.native", "message {}", text);
  logger->flush();

#ifdef _WIN32
  EXPECT_EQ(stream->str(), "test.native|info|message percent % and braces {}\r\n");
#else
  EXPECT_EQ(stream->str(), "test.native|info|message percent % and braces {}\n");
#endif
}

TEST(TesseractLoggingUnit, UsesNativeSpdlogFiltering)
{
  auto logger = tesseract::common::getLogger("test.filtering");
  auto stream = std::make_shared<std::ostringstream>();
  logger->sinks() = { std::make_shared<spdlog::sinks::ostream_sink_mt>(*stream) };
  logger->set_level(spdlog::level::err);

  TESSERACT_LOG_INFO_NAMED("test.filtering", "filtered");
  TESSERACT_LOG_ERROR_NAMED("test.filtering", "retained");
  logger->flush();

  EXPECT_EQ(stream->str().find("filtered"), std::string::npos);
  EXPECT_NE(stream->str().find("retained"), std::string::npos);
}

TEST(TesseractLoggingUnit, PreservesStructuredFieldsForHandlers)
{
  std::vector<tesseract::common::LogRecord> records;
  const auto handler_id = tesseract::common::addLogRecordHandler(
      [&records](const tesseract::common::LogRecord& record) { records.push_back(record); });
  ASSERT_NE(handler_id, 0);

  auto logger = tesseract::common::getLogger("test.structured");
  logger->set_level(spdlog::level::trace);

  tesseract::common::LogRecord record;
  record.level = spdlog::level::warn;
  record.logger_name = logger->name();
  record.component_name = "environment";
  record.message = "changed";
  record.attributes.emplace("attempt", std::int64_t{ 3 });
  record.attributes.emplace("object_id", std::string("robot_1"));
  record.attributes.emplace("successful", true);
  record.attributes.emplace("ratio", 0.5);
  record.attributes.emplace("name", std::string("value"));
  tesseract::common::emitLogRecord(record);

  EXPECT_TRUE(tesseract::common::removeLogRecordHandler(handler_id));
  ASSERT_EQ(records.size(), 1);
  EXPECT_EQ(records.front().component_name, "environment");
  EXPECT_EQ(std::get<std::int64_t>(records.front().attributes.at("attempt")), 3);
  EXPECT_EQ(std::get<std::string>(records.front().attributes.at("object_id")), "robot_1");
  EXPECT_EQ(std::get<bool>(records.front().attributes.at("successful")), true);
  EXPECT_DOUBLE_EQ(std::get<double>(records.front().attributes.at("ratio")), 0.5);
  EXPECT_EQ(std::get<std::string>(records.front().attributes.at("name")), "value");
}

TEST(TesseractLoggingUnit, HandlerMayRemoveItself)
{
  std::atomic_int calls{ 0 };
  tesseract::common::LogRecordHandlerId handler_id{ 0 };
  handler_id = tesseract::common::addLogRecordHandler([&](const auto&) {
    ++calls;
    tesseract::common::removeLogRecordHandler(handler_id);
  });
  ASSERT_NE(handler_id, 0);

  tesseract::common::emitLogRecord({});
  tesseract::common::emitLogRecord({});

  EXPECT_EQ(calls, 1);
}

TEST(TesseractLoggingUnit, HandlerExceptionsAreIsolated)
{
  std::atomic_int calls{ 0 };
  const auto throwing_handler =
      tesseract::common::addLogRecordHandler([](const auto&) { throw std::runtime_error("handler failure"); });
  const auto observing_handler = tesseract::common::addLogRecordHandler([&](const auto&) { ++calls; });
  ASSERT_NE(throwing_handler, 0);
  ASSERT_NE(observing_handler, 0);

  tesseract::common::emitLogRecord({});

  EXPECT_TRUE(tesseract::common::removeLogRecordHandler(throwing_handler));
  EXPECT_TRUE(tesseract::common::removeLogRecordHandler(observing_handler));
  EXPECT_EQ(calls, 1);
}

TEST(TesseractLoggingUnit, ConcurrentDispatchInvokesHandlerForEveryRecord)
{
  DefaultLoggerGuard guard;
  auto logger = std::make_shared<spdlog::logger>("tesseract", std::make_shared<spdlog::sinks::null_sink_mt>());
  logger->set_level(spdlog::level::info);
  tesseract::common::setLogger(std::move(logger));

  std::atomic<std::size_t> calls{ 0 };
  const auto handler_id = tesseract::common::addLogRecordHandler([&calls](const auto&) { ++calls; });
  ASSERT_NE(handler_id, 0);

  constexpr std::size_t thread_count = 8;
  static constexpr std::size_t records_per_thread = 1000;
  std::vector<std::thread> threads;
  threads.reserve(thread_count);
  for (std::size_t thread_index = 0; thread_index < thread_count; ++thread_index)
  {
    threads.emplace_back([] {
      for (std::size_t record_index = 0; record_index < records_per_thread; ++record_index)
        TESSERACT_LOG_INFO("Record {}", record_index);
    });
  }

  for (auto& thread : threads)
    thread.join();

  EXPECT_TRUE(tesseract::common::removeLogRecordHandler(handler_id));
  EXPECT_EQ(calls, thread_count * records_per_thread);
}

TEST(TesseractLoggingUnit, RemovedHandlerIsSkippedByInFlightDispatch)
{
  std::mutex mutex;
  std::condition_variable condition;
  bool first_handler_entered{ false };
  bool release_first_handler{ false };
  std::atomic_int removed_handler_calls{ 0 };

  const auto blocking_handler = tesseract::common::addLogRecordHandler([&](const auto&) {
    std::unique_lock<std::mutex> lock(mutex);
    first_handler_entered = true;
    condition.notify_one();
    condition.wait(lock, [&] { return release_first_handler; });
  });
  const auto removed_handler = tesseract::common::addLogRecordHandler([&](const auto&) { ++removed_handler_calls; });
  ASSERT_NE(blocking_handler, 0);
  ASSERT_NE(removed_handler, 0);

  std::thread dispatch_thread([] { tesseract::common::emitLogRecord({}); });
  {
    std::unique_lock<std::mutex> lock(mutex);
    condition.wait(lock, [&] { return first_handler_entered; });
  }
  EXPECT_TRUE(tesseract::common::removeLogRecordHandler(removed_handler));
  {
    std::lock_guard<std::mutex> lock(mutex);
    release_first_handler = true;
  }
  condition.notify_one();
  dispatch_thread.join();

  EXPECT_TRUE(tesseract::common::removeLogRecordHandler(blocking_handler));
  EXPECT_EQ(removed_handler_calls, 0);
}