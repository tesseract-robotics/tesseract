/**
 * @file tesseract_collision_configuration.h
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
#ifndef TESSERACT_COLLISION_TESSERACT_COLLISION_CONFIGURATION_H
#define TESSERACT_COLLISION_TESSERACT_COLLISION_CONFIGURATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_collision::tesseract_collision_bullet
{
/**
 * @brief This is a modified configuration that included the modified Bullet algorithms.
 *
 * This swaps out the following Bullet algorithms for modifed one:
 *     - Compound to Collision
 *     - Compound to Compound
 *     - Convex to Convex
 */
class TesseractCollisionConfiguration : public btDefaultCollisionConfiguration
{
public:
  TesseractCollisionConfiguration(
      const btDefaultCollisionConstructionInfo& constructionInfo = btDefaultCollisionConstructionInfo());
};
}  // namespace tesseract_collision::tesseract_collision_bullet
#endif  // TESSERACT_COLLISION_TESSERACT_COLLISION_CONFIGURATION_H
