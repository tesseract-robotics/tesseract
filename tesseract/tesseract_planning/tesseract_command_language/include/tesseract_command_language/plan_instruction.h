#ifndef TESSERACT_COMMAND_LANGUAGE_PLAN_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_PLAN_INSTRUCTION_H

#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/instruction_type.h>
#include <vector>
#include <Eigen/Geometry>

namespace tesseract_planning
{

enum class PlanInstructionType : int
{
  LINEAR,
  FREESPACE,
  CIRCULAR
};

class PlanInstruction
{
public:
  using Ptr = std::shared_ptr<PlanInstruction>;
  using ConstPtr = std::shared_ptr<const PlanInstruction>;

  PlanInstruction(Waypoint waypoint, PlanInstructionType type, std::string profile = "DEFAULT");

  void setWaypoint(Waypoint waypoint);
  const Waypoint& getWaypoint() const;

  void setTCP(Eigen::Isometry3d tcp);
  const Eigen::Isometry3d& getTCP() const;

  void setWorkingFrame(std::string working_frame);
  const std::string& getWorkingFrame() const;

  void setProfile(const std::string& profile);
  const std::string& getProfile() const;

  int getType() const;

  const std::string& getDescription() const;

  void setDescription(const std::string& description);

  bool isComposite() const;

  bool isPlan() const;

  bool isMove() const;

  void print() const;

  bool isLinear() const;

  bool isFreespace() const;

  bool isCircular() const;

private:
  int type_ { static_cast<int>(InstructionType::PLAN_INSTRUCTION) };

  PlanInstructionType plan_type_;

  /** @brief The assigned waypoint (Cartesian or Joint) */
  Waypoint waypoint_;

  /** @brief The tool center point */
  Eigen::Isometry3d tcp_ { Eigen::Isometry3d::Identity() };

  /** @brief The working frame the waypoint is relative to */
  std::string working_frame_;

  /** @brief The description of the instruction */
  std::string description_ {"Tesseract Plan Instruction"};

  /** @brief The profile used for this plan instruction */
  std::string profile_ {"DEFAULT"};

};

}

#endif // TESSERACT_COMMAND_LANGUAGE_PLAN_INSTRUCTION_H
