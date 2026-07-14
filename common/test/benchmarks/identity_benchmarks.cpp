#include <benchmark/benchmark.h>

#include <tesseract/common/allowed_collision_matrix.h>
#include <tesseract/common/collision_margin_data.h>
#include <tesseract/common/types.h>

#include <string>
#include <unordered_map>
#include <vector>

using tesseract::common::AllowedCollisionMatrix;
using tesseract::common::CollisionMarginPairData;
using tesseract::common::LinkId;
using tesseract::common::LinkIdPair;

namespace
{
// Realistic robot-link-name lengths (beyond SSO) so string costs are representative.
std::vector<LinkId> makeLinkIds(std::size_t n)
{
  std::vector<LinkId> ids;
  ids.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
    ids.emplace_back("robot_arm_segment_link_" + std::to_string(i));
  return ids;
}

AllowedCollisionMatrix makeAcm(const std::vector<LinkId>& ids)
{
  AllowedCollisionMatrix acm;
  for (std::size_t a = 0; a < ids.size(); ++a)
    for (std::size_t b = a + 1; b < ids.size(); ++b)
      acm.addAllowedCollision(ids[a], ids[b], "adjacent");
  return acm;
}
}  // namespace

// Cost of building a pair key from two held ids (bullet/fcl narrowphase shape).
static void BM_PairConstruction(benchmark::State& state)
{
  const auto ids = makeLinkIds(64);
  std::size_t i = 0;
  for (auto _ : state)
  {
    LinkIdPair pair(ids[i & 63U], ids[(i + 7) & 63U]);
    benchmark::DoNotOptimize(pair);
    ++i;
  }
}
BENCHMARK(BM_PairConstruction);

// Cost of refilling a long-lived scratch pair (thread_local + assign shape).
static void BM_PairAssign(benchmark::State& state)
{
  const auto ids = makeLinkIds(64);
  LinkIdPair pair;
  std::size_t i = 0;
  for (auto _ : state)
  {
    pair.assign(ids[i & 63U], ids[(i + 7) & 63U]);
    benchmark::DoNotOptimize(pair);
    ++i;
  }
}
BENCHMARK(BM_PairAssign);

// ACM hit with a preconstructed pair (construct-once-reuse shape).
static void BM_AcmIsAllowed_PreconstructedPair_Hit(benchmark::State& state)
{
  const auto ids = makeLinkIds(64);
  const auto acm = makeAcm(ids);
  std::vector<LinkIdPair> keys;
  for (std::size_t a = 0; a < ids.size(); ++a)
    for (std::size_t b = a + 1; b < ids.size(); ++b)
      keys.emplace_back(ids[a], ids[b]);
  std::size_t i = 0;
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(acm.isCollisionAllowed(keys[i % keys.size()]));
    ++i;
  }
}
BENCHMARK(BM_AcmIsAllowed_PreconstructedPair_Hit);

// ACM miss with a preconstructed pair.
static void BM_AcmIsAllowed_PreconstructedPair_Miss(benchmark::State& state)
{
  const auto ids = makeLinkIds(64);
  const auto acm = makeAcm(ids);
  const auto outsiders = makeLinkIds(8);  // same names? No — regenerate with a distinct prefix:
  std::vector<LinkIdPair> miss_keys;
  miss_keys.reserve(outsiders.size());
  for (std::size_t a = 0; a < outsiders.size(); ++a)
    miss_keys.emplace_back(LinkId("unrelated_object_" + std::to_string(a)), ids[a]);
  std::size_t i = 0;
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(acm.isCollisionAllowed(miss_keys[i % miss_keys.size()]));
    ++i;
  }
}
BENCHMARK(BM_AcmIsAllowed_PreconstructedPair_Miss);

// ACM hit via the two-id overload (gradient-eval shape; interim fat-pair cost shows up here).
static void BM_AcmIsAllowed_TwoIds_Hit(benchmark::State& state)
{
  const auto ids = makeLinkIds(64);
  const auto acm = makeAcm(ids);
  std::size_t i = 0;
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(acm.isCollisionAllowed({ ids[i & 63U], ids[(i + 7) & 63U] }));
    ++i;
  }
}
BENCHMARK(BM_AcmIsAllowed_TwoIds_Hit);

// Margin lookup via two ids (the collision_terms.cpp hot shape).
static void BM_MarginLookup_TwoIds(benchmark::State& state)
{
  const auto ids = makeLinkIds(64);
  CollisionMarginPairData margins;
  for (std::size_t a = 0; a < ids.size(); ++a)
    for (std::size_t b = a + 1; b < ids.size(); ++b)
      margins.setCollisionMargin(ids[a], ids[b], 0.01);
  std::size_t i = 0;
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(margins.getCollisionMargin(ids[i & 63U], ids[(i + 7) & 63U]));
    ++i;
  }
}
BENCHMARK(BM_MarginLookup_TwoIds);

// Pair-keyed map population (ContactResultMap-shaped write path: fresh keys inserted per query).
static void BM_PairMapPopulate(benchmark::State& state)
{
  const auto ids = makeLinkIds(64);
  for (auto _ : state)
  {
    std::unordered_map<LinkIdPair, int> map;
    map.reserve(64);
    for (std::size_t a = 0; a < 32; ++a)
      map.emplace(LinkIdPair(ids[a], ids[a + 32]), static_cast<int>(a));
    benchmark::DoNotOptimize(map);
  }
}
BENCHMARK(BM_PairMapPopulate);

BENCHMARK_MAIN();
