/**
 * @file descartes_vertex_evaluator.h
 * @brief Tesseract Descartes Vertex Evaluator
 *
 * @author Levi Armstrong
 * @date June 25, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_VERTEX_EVALUATOR_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_VERTEX_EVALUATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
template <typename FloatType>
class DescartesVertexEvaluator
{
public:
  using Ptr = typename std::shared_ptr<DescartesVertexEvaluator<FloatType>>;
  DescartesVertexEvaluator() = default;
  virtual ~DescartesVertexEvaluator() = default;
  DescartesVertexEvaluator(const DescartesVertexEvaluator&) = delete;
  DescartesVertexEvaluator& operator=(const DescartesVertexEvaluator&) = delete;
  DescartesVertexEvaluator(DescartesVertexEvaluator&&) = delete;
  DescartesVertexEvaluator& operator=(DescartesVertexEvaluator&&) = delete;

  /**
   * @brief Determines whether the vertex is valid and, if so, its cost.
   * @param vertex The vertex to consider
   * @return True if vertex is valid, false otherwise. The relative cost associated with the vertex
   */
  virtual bool operator()(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& vertex) const = 0;
};

template <typename FloatType>
class DescartesJointLimitsVertexEvaluator : public DescartesVertexEvaluator<FloatType>
{
public:
  using Ptr = typename std::shared_ptr<DescartesJointLimitsVertexEvaluator<FloatType>>;
  DescartesJointLimitsVertexEvaluator(Eigen::MatrixX2d limits) : limits_(std::move(limits)) {}

  bool operator()(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& vertex) const override
  {
    assert(limits_.rows() == vertex.size());

    for (int i = 0; i < limits_.rows(); ++i)
      if ((vertex(i) < limits_(i, 0)) || (vertex(i) > limits_(i, 1)))
        return false;

    return true;
  }

protected:
  Eigen::MatrixX2d limits_;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_VERTEX_EVALUATOR_H
