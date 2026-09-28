#include <benchmark/benchmark.h>

#include <tesseract/common/logging.h>

#include <spdlog/sinks/null_sink.h>

#include <memory>

namespace
{
spdlog::logger& getBenchmarkLogger()
{
  static const auto logger = [] {
    auto benchmark_logger =
        std::make_shared<spdlog::logger>("tesseract", std::make_shared<spdlog::sinks::null_sink_mt>());
    benchmark_logger->set_level(spdlog::level::info);
    tesseract::common::setLogger(benchmark_logger);
    return benchmark_logger;
  }();
  return *logger;
}

tesseract::common::LogRecordHandlerId& getBenchmarkHandlerId()
{
  static tesseract::common::LogRecordHandlerId handler_id{ 0 };
  return handler_id;
}

void setupNoOpHandler(const benchmark::State&)
{
  (void)getBenchmarkLogger();
  getBenchmarkHandlerId() = tesseract::common::addLogRecordHandler([](const auto&) {});
}

void teardownNoOpHandler(const benchmark::State&)
{
  tesseract::common::removeLogRecordHandler(getBenchmarkHandlerId());
  getBenchmarkHandlerId() = 0;
}

void BM_DefaultMacroDisabled(benchmark::State& state)
{
  (void)getBenchmarkLogger();
  for (auto _ : state)  // NOLINT
  {
    TESSERACT_LOG_DEBUG("Value {}", 42);
  }
}
BENCHMARK(BM_DefaultMacroDisabled)->ThreadRange(1, 16);

void BM_LevelCheckDisabled(benchmark::State& state)
{
  (void)getBenchmarkLogger();
  for (auto _ : state)  // NOLINT
  {
    benchmark::DoNotOptimize(tesseract::common::isLogLevelEnabled(spdlog::level::debug));
  }
}
BENCHMARK(BM_LevelCheckDisabled)->ThreadRange(1, 16);

void BM_CachedNativeSpdlogDisabled(benchmark::State& state)
{
  auto& logger = getBenchmarkLogger();
  for (auto _ : state)  // NOLINT
  {
    logger.debug("Value {}", 42);
  }
}
BENCHMARK(BM_CachedNativeSpdlogDisabled)->ThreadRange(1, 16);

void BM_DefaultMacroEnabledNoHandlers(benchmark::State& state)
{
  (void)getBenchmarkLogger();
  for (auto _ : state)  // NOLINT
  {
    TESSERACT_LOG_INFO("Value {}", 42);
  }
}
BENCHMARK(BM_DefaultMacroEnabledNoHandlers)->ThreadRange(1, 16);

void BM_DefaultMacroEnabledNoOpHandler(benchmark::State& state)
{
  for (auto _ : state)  // NOLINT
  {
    TESSERACT_LOG_INFO("Value {}", 42);
  }
}
BENCHMARK(BM_DefaultMacroEnabledNoOpHandler)
    ->Setup(setupNoOpHandler)
    ->Teardown(teardownNoOpHandler)
    ->ThreadRange(1, 16);

void BM_ResolvedRecordEmptyAttributes(benchmark::State& state)
{
  auto& logger = getBenchmarkLogger();
  tesseract::common::LogRecord record;
  record.message = "Event";
  for (auto _ : state)  // NOLINT
  {
    tesseract::common::detail::emitLogRecord(logger, record);
  }
}
BENCHMARK(BM_ResolvedRecordEmptyAttributes)->ThreadRange(1, 16);

void BM_ResolvedRecordPopulatedAttributes(benchmark::State& state)
{
  auto& logger = getBenchmarkLogger();
  tesseract::common::LogRecord record;
  record.message = "Event";
  record.attributes.emplace("attempt", std::int64_t{ 3 });
  record.attributes.emplace("object_id", std::string("robot_1"));
  record.attributes.emplace("ratio", 0.5);
  record.attributes.emplace("successful", true);
  for (auto _ : state)  // NOLINT
  {
    tesseract::common::detail::emitLogRecord(logger, record);
  }
}
BENCHMARK(BM_ResolvedRecordPopulatedAttributes)->ThreadRange(1, 16);
}  // namespace

BENCHMARK_MAIN();