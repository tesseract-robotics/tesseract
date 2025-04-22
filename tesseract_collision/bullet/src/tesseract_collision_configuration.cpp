/**
 * @file tesseract_collision_configuration.cpp
 * @brief Modified bullet collision configuration
 *
 * @author Levi Armstrong
 * @date April 1, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (BSD-2-Clause)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h>
#include <LinearMath/btPoolAllocator.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/bullet/tesseract_collision_configuration.h>
#include <tesseract_collision/bullet/tesseract_compound_collision_algorithm.h>
#include <tesseract_collision/bullet/tesseract_compound_compound_collision_algorithm.h>
#include <tesseract_collision/bullet/tesseract_convex_convex_algorithm.h>

namespace tesseract_collision::tesseract_collision_bullet
{
TesseractCollisionConfigurationInfo::TesseractCollisionConfigurationInfo(bool create_pool_allocators,
                                                                         bool share_pool_allocators)
  : share_pool_allocators_(share_pool_allocators)
{
  if (create_pool_allocators)
    createPoolAllocators();
}

TesseractCollisionConfigurationInfo TesseractCollisionConfigurationInfo::clone() const
{
  if (persistent_manifold_pool_ == nullptr || collision_algorithm_pool_ == nullptr)
    throw std::runtime_error("Pool allocators have not been setup!");

  if (share_pool_allocators_)
    return *this;

  TesseractCollisionConfigurationInfo copy(*this);
  copy.createPoolAllocators();

  assert(copy.share_pool_allocators_ == share_pool_allocators_);
  assert(copy.collision_algorithm_pool_ != collision_algorithm_pool_);
  assert(copy.persistent_manifold_pool_ != persistent_manifold_pool_);

  return copy;
}

void TesseractCollisionConfigurationInfo::createPoolAllocators()
{
  persistent_manifold_pool_ =
      std::make_shared<btPoolAllocator>(sizeof(btPersistentManifold), m_defaultMaxPersistentManifoldPoolSize);
  m_persistentManifoldPool = persistent_manifold_pool_.get();

  /// calculate maximum element size, big enough to fit any collision algorithm in the memory pool
  int maxSize = sizeof(TesseractConvexConvexAlgorithm);
  int maxSize2 = sizeof(btConvexConcaveCollisionAlgorithm);
  int maxSize3 = sizeof(TesseractCompoundCollisionAlgorithm);
  int maxSize4 = sizeof(TesseractCompoundCompoundCollisionAlgorithm);

  int collisionAlgorithmMaxElementSize = btMax(maxSize, m_customCollisionAlgorithmMaxElementSize);
  collisionAlgorithmMaxElementSize = btMax(collisionAlgorithmMaxElementSize, maxSize2);
  collisionAlgorithmMaxElementSize = btMax(collisionAlgorithmMaxElementSize, maxSize3);
  collisionAlgorithmMaxElementSize = btMax(collisionAlgorithmMaxElementSize, maxSize4);

  TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
  collisionAlgorithmMaxElementSize = (collisionAlgorithmMaxElementSize + 16) & 0xffffffffffff0;  // NOLINT
  TESSERACT_COMMON_IGNORE_WARNINGS_POP

  collision_algorithm_pool_ =
      std::make_shared<btPoolAllocator>(collisionAlgorithmMaxElementSize, m_defaultMaxCollisionAlgorithmPoolSize);
  m_collisionAlgorithmPool = collision_algorithm_pool_.get();
}

TesseractCollisionConfiguration::TesseractCollisionConfiguration(const TesseractCollisionConfigurationInfo& config_info)
  : btDefaultCollisionConfiguration(config_info)
{
  assert(config_info.m_collisionAlgorithmPool != nullptr);
  assert(config_info.m_persistentManifoldPool != nullptr);
  assert(m_ownsCollisionAlgorithmPool == false);
  assert(m_ownsPersistentManifoldPool == false);

  void* mem = nullptr;

  m_compoundCreateFunc->~btCollisionAlgorithmCreateFunc();
  btAlignedFree(m_compoundCreateFunc);

  m_compoundCompoundCreateFunc->~btCollisionAlgorithmCreateFunc();
  btAlignedFree(m_compoundCompoundCreateFunc);

  m_swappedCompoundCreateFunc->~btCollisionAlgorithmCreateFunc();
  btAlignedFree(m_swappedCompoundCreateFunc);

  m_convexConvexCreateFunc->~btCollisionAlgorithmCreateFunc();
  btAlignedFree(m_convexConvexCreateFunc);

  if (m_ownsCollisionAlgorithmPool)
  {
    m_collisionAlgorithmPool->~btPoolAllocator();
    btAlignedFree(m_collisionAlgorithmPool);
  }
  if (m_ownsPersistentManifoldPool)
  {
    m_persistentManifoldPool->~btPoolAllocator();
    btAlignedFree(m_persistentManifoldPool);
  }

  mem = btAlignedAlloc(sizeof(TesseractConvexConvexAlgorithm::CreateFunc), 16);
  m_convexConvexCreateFunc = new (mem) TesseractConvexConvexAlgorithm::CreateFunc(m_pdSolver);  // NOLINT

  mem = btAlignedAlloc(sizeof(TesseractCompoundCollisionAlgorithm::CreateFunc), 16);
  m_compoundCreateFunc = new (mem) TesseractCompoundCollisionAlgorithm::CreateFunc;  // NOLINT

  mem = btAlignedAlloc(sizeof(TesseractCompoundCompoundCollisionAlgorithm::CreateFunc), 16);
  m_compoundCompoundCreateFunc = new (mem) TesseractCompoundCompoundCollisionAlgorithm::CreateFunc;  // NOLINT

  mem = btAlignedAlloc(sizeof(TesseractCompoundCollisionAlgorithm::SwappedCreateFunc), 16);
  m_swappedCompoundCreateFunc = new (mem) TesseractCompoundCollisionAlgorithm::SwappedCreateFunc;  // NOLINT
}

}  // namespace tesseract_collision::tesseract_collision_bullet
