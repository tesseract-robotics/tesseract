#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_process_managers/process_generators/fix_state_collision_process_generator.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>

#include "freespace_example_program.h"

using namespace tesseract_kinematics;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_planning;
using namespace tesseract_collision;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

class FixStateCollisionProcessGeneratorUnit : public ::testing::Test
{
protected:
  Environment::Ptr env_;
  ManipulatorInfo manip_;

  void SetUp() override
  {
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    Environment::Ptr env = std::make_shared<Environment>();

    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/boxbot.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/boxbot.srdf");
    EXPECT_TRUE(env->init<OFKTStateSolver>(urdf_path, srdf_path, locator));
    env_ = env;

    manip_.manipulator = "manipulator";
  }
};

TEST_F(FixStateCollisionProcessGeneratorUnit, StateInCollisionTest)
{
  CompositeInstruction program = freespaceExampleProgramABB();
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);
  ProcessInput input(env_, &program_instruction, manip_, &seed, false, nullptr);

  FixStateCollisionProfile profile;

  Eigen::VectorXd state = Eigen::VectorXd::Zero(2);
  tesseract_collision::ContactResultMap contacts;
  EXPECT_TRUE(StateInCollision(state, input, profile, contacts));
  EXPECT_FALSE(contacts.empty());
  state[0] = 1.5;

  contacts.clear();
  EXPECT_FALSE(StateInCollision(state, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());
  state[0] = 0.0;
  state[1] = 1.5;

  contacts.clear();
  EXPECT_FALSE(StateInCollision(state, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());

  // Check that the safety margin is obeyed
  profile.collision_check_config.collision_margin_data = CollisionMarginData(0.1);
  state[0] = 0.0;
  state[1] = 1.05;
  contacts.clear();
  EXPECT_TRUE(StateInCollision(state, input, profile, contacts));
  EXPECT_FALSE(contacts.empty());

  profile.collision_check_config.collision_margin_data = CollisionMarginData(0.01);
  contacts.clear();
  EXPECT_FALSE(StateInCollision(state, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());
}

TEST_F(FixStateCollisionProcessGeneratorUnit, WaypointInCollisionTest)
{
  CompositeInstruction program = freespaceExampleProgramABB();
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);
  ProcessInput input(env_, &program_instruction, manip_, &seed, false, nullptr);

  FixStateCollisionProfile profile;

  Eigen::VectorXd state = Eigen::VectorXd::Zero(2);
  JointWaypoint waypoint({ "boxbot_x_joint", "boxbot_y_joint" }, state);
  tesseract_collision::ContactResultMap contacts;

  EXPECT_TRUE(WaypointInCollision(waypoint, input, profile, contacts));
  EXPECT_FALSE(contacts.empty());

  waypoint[0] = 1.5;
  contacts.clear();
  EXPECT_FALSE(WaypointInCollision(waypoint, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());

  waypoint[0] = 0.0;
  waypoint[1] = 1.5;
  contacts.clear();
  EXPECT_FALSE(WaypointInCollision(waypoint, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());

  // Check that the safety margin is obeyed
  profile.collision_check_config.collision_margin_data = CollisionMarginData(0.1);
  waypoint[0] = 0.0;
  waypoint[1] = 1.05;
  contacts.clear();
  EXPECT_TRUE(WaypointInCollision(waypoint, input, profile, contacts));
  EXPECT_FALSE(contacts.empty());

  profile.collision_check_config.collision_margin_data = CollisionMarginData(0.01);
  contacts.clear();
  EXPECT_FALSE(WaypointInCollision(waypoint, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());

  // Check that it catches invalid inputs correctly
  CartesianWaypoint cart_wp(Eigen::Isometry3d::Identity());
  contacts.clear();
  EXPECT_FALSE(WaypointInCollision(cart_wp, input, profile, contacts));
  EXPECT_TRUE(contacts.empty());
}

TEST_F(FixStateCollisionProcessGeneratorUnit, MoveWaypointFromCollisionRandomSamplerTest)
{
  CompositeInstruction program = freespaceExampleProgramABB();
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);
  ProcessInput input(env_, &program_instruction, manip_, &seed, false, nullptr);

  FixStateCollisionProfile profile;

  Eigen::VectorXd state = Eigen::VectorXd::Zero(2);
  JointWaypoint waypoint({ "boxbot_x_joint", "boxbot_y_joint" }, state);

  // Check that the safety margin is obeyed
  profile.collision_check_config.collision_margin_data = CollisionMarginData(0.1);
  profile.jiggle_factor = 1.0;
  waypoint[0] = 0.0;
  waypoint[1] = 1.09;
  Waypoint wp(waypoint);
  tesseract_collision::ContactResultMap contacts;

  // Attempts are 0, so it should still be in collision
  profile.sampling_attempts = 0;
  EXPECT_TRUE(WaypointInCollision(wp, input, profile, contacts));
  EXPECT_FALSE(MoveWaypointFromCollisionRandomSampler(wp, input, profile));
  EXPECT_TRUE(WaypointInCollision(wp, input, profile, contacts));

  // It is very unlikely that this will still fail
  profile.sampling_attempts = 1000;
  EXPECT_TRUE(MoveWaypointFromCollisionRandomSampler(wp, input, profile));
  EXPECT_FALSE(WaypointInCollision(wp, input, profile, contacts));
}

TEST_F(FixStateCollisionProcessGeneratorUnit, MoveWaypointFromCollisionTrajoptTest)
{
  CompositeInstruction program = freespaceExampleProgramABB();
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);
  ProcessInput input(env_, &program_instruction, manip_, &seed, false, nullptr);

  FixStateCollisionProfile profile;

  Eigen::VectorXd state = Eigen::VectorXd::Zero(2);
  JointWaypoint waypoint({ "boxbot_x_joint", "boxbot_y_joint" }, state);

  // Check that the safety margin is obeyed
  profile.collision_check_config.collision_margin_data = CollisionMarginData(0.1);
  profile.jiggle_factor = 1.0;
  waypoint[0] = 0.0;
  waypoint[1] = 1.09;
  Waypoint wp(waypoint);
  tesseract_collision::ContactResultMap contacts;

  EXPECT_TRUE(WaypointInCollision(wp, input, profile, contacts));
  EXPECT_TRUE(MoveWaypointFromCollisionTrajopt(wp, input, profile));
  EXPECT_FALSE(WaypointInCollision(wp, input, profile, contacts));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
