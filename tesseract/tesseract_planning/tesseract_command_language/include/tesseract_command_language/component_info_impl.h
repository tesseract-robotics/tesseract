#ifndef TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_IMPL_H
#define TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_IMPL_H

#include <Eigen/Geometry>
#include <vector>
#include <tesseract_collision/core/types.h>
#include <trajopt/utils.hpp>
#include <trajopt/problem_description.hpp>

namespace tesseract_planning
{

enum class ComponentTypes : int
{
  FIXED,
  CARTESIAN_X_TOL,
  CARTESIAN_Y_TOL,
  CARTESIAN_Z_TOL,
  CARTESIAN_XY_TOL,
  CARTESIAN_XZ_TOL,
  CARTESIAN_YZ_TOL,
  CARTESIAN_XYZ_TOL,
  JOINT_TOL,
  VELOCITY_TOL,
  AVOID_SINGULARITY,
  NEAR_JOINT_STATE,
  VELOCITY_SMOOTHING,
  ACCELERATION_SMOOTHING,
  JERK_SMOOTHING,
  AVOID_COLLISION
};

class FixedComponentInfo
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::FIXED); }
  int getMask() const { return mask; }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return false; }

  /** @brief This is the coefficient/weight that may be used by the planner */
  Eigen::VectorXd coeff { Eigen::VectorXd::Constant(1, 1, 1) };

  /** @brief The name of the component */
  std::string name {"Fixed Component"};

  /** @brief A mask to allow it to only be used for a certain planner */
  int mask {-1};
};

class CartesianXTolComponentInfo
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::CARTESIAN_X_TOL); }
  int getMask() const { return mask; }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  double target{0};
  double min{0};
  double max{0};

  /** @brief This is the coefficient/weight that may be used by the planner */
  double coeff {1};

  /** @brief The name of the component */
  std::string name {"Cartesian X Tolerance Component"};

  /** @brief A mask to allow it to only be used for a certain planner */
  int mask {-1};
};

class VelocityComponentInfo
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::VELOCITY_TOL); }
  int getMask() const { return mask; }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  double target{0};
  double min{0};
  double max{0};

  /** @brief This is the coefficient/weight that may be used by the planner */
  double coeff {1};

  /** @brief The name of the component */
  std::string name {"Velocity Component"};

  /** @brief A mask to allow it to only be used for a certain planner */
  int mask {-1};
};

class VelocitySmoothingComponentInfo
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::VELOCITY_SMOOTHING); }
  int getMask() const { return mask; }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  /** @brief This is the coefficient/weight that may be used by the planner */
  Eigen::VectorXd coeff { Eigen::VectorXd::Constant(1, 1, 1) };

  /** @brief The name of the component */
  std::string name {"Velocity Smoothing Component"};

  /** @brief A mask to allow it to only be used for a certain planner */
  int mask {-1};
};

class AccelerationSmoothingComponentInfo
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::ACCELERATION_SMOOTHING); }
  int getMask() const { return mask; }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  /** @brief This is the coefficient/weight that may be used by the planner */
  Eigen::VectorXd coeff { Eigen::VectorXd::Constant(1, 1, 1) };

  /** @brief The name of the component */
  std::string name {"Acceleration Smoothing Component"};

  /** @brief A mask to allow it to only be used for a certain planner */
  int mask {-1};
};

class JerkSmoothingComponentInfo
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::JERK_SMOOTHING); }
  int getMask() const { return mask; }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  /** @brief This is the coefficient/weight that may be used by the planner */
  Eigen::VectorXd coeff { Eigen::VectorXd::Constant(1, 1, 1) };

  /** @brief The name of the component */
  std::string name {"Jerk Smoothing Component"};

  /** @brief A mask to allow it to only be used for a certain planner */
  int mask {-1};
};

/** @brief This component tells the planner to avoid singularity for a given move instruction */
class AvoidSingularityComponentInfo
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::AVOID_SINGULARITY); }
  int getMask() const { return mask; }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  /** @brief This is the coefficient/weight that may be used by the planner */
  Eigen::VectorXd coeff { Eigen::VectorXd::Constant(1, 1, 1) };

  /** @brief The name of the component */
  std::string name {"Avoid Singularity Component"};

  /** @brief A mask to allow it to only be used for a certain planner */
  int mask {-1};
};

/** @brief This component tells the planner to stay near a joint target for a given move instruction  */
class NearJointStateComponentInfo
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::NEAR_JOINT_STATE); }
  int getMask() const { return mask; }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  /** @brief The joint state target to stay near for a given move instruction */
  Eigen::VectorXd target;

  /** @brief The joint names for the provided target */
  std::vector<std::string> joint_names;

  /** @brief This is the coefficient/weight that may be used by the planner */
  Eigen::VectorXd coeff { Eigen::VectorXd::Constant(1, 1, 1) };

  /** @brief The name of the component */
  std::string name {"Near Joint State Component"};

  /** @brief A mask to allow it to only be used for a certain planner */
  int mask {-1};
};

/** @brief This component tells the planner to stay near a joint target for a given move instruction  */
class AvoidCollisionComponentInfo
{
public:
  int getType() const { return static_cast<int>(ComponentTypes::AVOID_COLLISION); }
  int getMask() const { return mask; }
  const std::string& getName() const { return name; }
  bool isCompositeInstructionSupported() const { return true; }

  /** @brief Indicate the type of collision checking that should be used. */
  trajopt::CollisionEvaluatorType evaluator_type {trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP};

  /**
   * @brief Use the weighted sum for each link pair. This reduces the number equations added to the problem
   * When enable it is good to start with a coefficient of 1 otherwise 20 is a good starting point.
   */
  bool use_weighted_sum {false};

  /** @brief Set the resolution at which state validity needs to be verified in order for a motion between two states
   * to be considered valid. If norm(state1 - state0) > longest_valid_segment_length.
   *
   * Note: This gets converted to longest_valid_segment_fraction.
   *       longest_valid_segment_fraction = longest_valid_segment_length / state_space.getMaximumExtent()
   */
  double longest_valid_segment_length {0.5};

  /** @brief Max distance in which collision constraints will be evaluated. */
  double safety_margin {0.025};

  /** @brief A buffer added to the collision margin distance. Contact results that are within the safety margin buffer
  distance but greater than the safety margin distance (i.e. close but not in collision) will be evaluated but will not
  contribute costs to the optimization problem. This helps keep the solution away from collision constraint conditions
  when the safety margin distance is small.*/
  double safety_margin_buffer {0.05};

  /** @brief The collision coeff/weight */
  double coeff {20};

  /** @brief Set the contact test type that should be used. */
  tesseract_collision::ContactTestType contact_test_type {tesseract_collision::ContactTestType::ALL};

  /** @brief The name of the component */
  std::string name {"Avoid Collision Component"};

  /** @brief A mask to allow it to only be used for a certain planner */
  int mask {-1};
};
}
#endif // TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_IMPL_H
