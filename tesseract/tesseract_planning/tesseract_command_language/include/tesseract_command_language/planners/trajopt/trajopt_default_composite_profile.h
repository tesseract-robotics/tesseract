#ifndef TESSERACT_COMMAND_LANGUAGE_TRAJOPT_DEFAULT_COMPOSITE_PROFILE_H
#define TESSERACT_COMMAND_LANGUAGE_TRAJOPT_DEFAULT_COMPOSITE_PROFILE_H

#include <tesseract_motion_planners/trajopt/config/trajopt_collision_config.h>
#include <tesseract_command_language/planners/trajopt/trajopt_profile.h>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Core>

namespace tesseract_planning
{
class TrajOptDefaultCompositeProfile : public TrajOptCompositeProfile
{
public:

  /** @brief The type of contact test to perform: FIRST, CLOSEST, ALL */
  tesseract_collision::ContactTestType contact_test_type = tesseract_collision::ContactTestType::ALL;
  /** @brief Configuration info for collisions that are modeled as costs */
  tesseract_motion_planners::CollisionCostConfig collision_cost_config;
  /** @brief Configuration info for collisions that are modeled as constraints */
  tesseract_motion_planners::CollisionConstraintConfig collision_constraint_config;
  /** @brief If true, a joint velocity cost with a target of 0 will be applied for all timesteps Default: true*/
  bool smooth_velocities = true;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd velocity_coeff;
  /** @brief If true, a joint acceleration cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_accelerations = true;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd acceleration_coeff;
  /** @brief If true, a joint jerk cost with a target of 0 will be applied for all timesteps Default: false*/
  bool smooth_jerks = true;
  /** @brief This default to all ones, but allows you to weight different joints */
  Eigen::VectorXd jerk_coeff;
  /** @brief If true, applies a cost to avoid kinematic singularities */
  bool avoid_singularity = false;
  /** @brief Optimization weight associated with kinematic singularity avoidance */
  double avoid_singularity_coeff = 5.0;

  /**@brief Special link collision cost distances */
  trajopt::SafetyMarginData::Ptr special_collision_cost{ nullptr };
  /**@brief Special link collision constraint distances */
  trajopt::SafetyMarginData::Ptr special_collision_constraint{ nullptr };

  /** @brief Set the resolution at which state validity needs to be verified in order for a motion between two states
   * to be considered valid in post checking of trajectory returned by trajopt.
   *
   * The resolution is equal to longest_valid_segment_fraction * state_space.getMaximumExtent()
   *
   * Note: The planner takes the conservative of either longest_valid_segment_fraction or longest_valid_segment_length.
   */
  double longest_valid_segment_fraction = 0.01;  // 1%

  /** @brief Set the resolution at which state validity needs to be verified in order for a motion between two states
   * to be considered valid. If norm(state1 - state0) > longest_valid_segment_length.
   *
   * Note: This gets converted to longest_valid_segment_fraction.
   *       longest_valid_segment_fraction = longest_valid_segment_length / state_space.getMaximumExtent()
   */
  double longest_valid_segment_length = 0.5;

  void apply(trajopt::ProblemConstructionInfo& pci,
             int start_index,
             int end_index,
             const std::vector<std::string>& active_links,
             const std::vector<int>& fixed_indices) override;

protected:

  void addCollisionCost(trajopt::ProblemConstructionInfo& pci,
                        int start_index, int end_index,
                        const std::vector<int>& fixed_indices) const;

  void addCollisionConstraint(trajopt::ProblemConstructionInfo& pci,
                              int start_index, int end_index,
                              const std::vector<int>& fixed_indices) const;

  void addVelocitySmoothing(trajopt::ProblemConstructionInfo& pci,
                            int start_index, int end_index,
                            const std::vector<int>& fixed_indices) const;

  void addAccelerationSmoothing(trajopt::ProblemConstructionInfo& pci,
                                int start_index, int end_index,
                                const std::vector<int>& fixed_indices) const;

  void addJerkSmoothing(trajopt::ProblemConstructionInfo& pci,
                        int start_index, int end_index,
                        const std::vector<int>& fixed_indices) const;

  void addConstraintErrorFunctions(trajopt::ProblemConstructionInfo& pci,
                                   int start_index, int end_index,
                                   const std::vector<int>& fixed_indices) const;

  void addAvoidSingularity(trajopt::ProblemConstructionInfo& pci,
                           int start_index, int end_index,
                           const std::string& link,
                           const std::vector<int>& fixed_indices) const;
};
}

#endif // TESSERACT_COMMAND_LANGUAGE_TRAJOPT_DEFAULT_COMPOSITE_PROFILE_H
