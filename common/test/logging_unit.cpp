#include <tesseract/common/logging.h>

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

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