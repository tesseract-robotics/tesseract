/**
 * @file tesseract_common_serialization_unit.cpp
 * @brief Tests serialization of types in tesseract_common
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
 * @version TODO
 * @bug No known bugs
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/nvp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/eigen_serialization.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/unit_test_utils.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/collision_margin_data.h>
#include <tesseract_common/atomic_serialization.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/manipulator_info.h>

using namespace tesseract_common;

TEST(TesseractCommonSerializeUnit, KinematicLimits)  // NOLINT
{
  KinematicLimits limits;
  limits.resize(3);
  EXPECT_EQ(limits.joint_limits.rows(), 3);
  EXPECT_EQ(limits.velocity_limits.rows(), 3);
  EXPECT_EQ(limits.acceleration_limits.rows(), 3);

  limits.joint_limits << -5, 5, -5, 5, -5, 5;
  limits.velocity_limits = Eigen::VectorXd::Constant(3, 6);
  limits.acceleration_limits = Eigen::VectorXd::Constant(3, 7);

  tesseract_common::testSerialization<KinematicLimits>(limits, "KinematicLimits");
}

TEST(TesseractCommonSerializeUnit, ManipulatorInfo)  // NOLINT
{
  ManipulatorInfo manip_info("manipulator", "world", "tool0");
  tesseract_common::testSerialization<ManipulatorInfo>(manip_info, "ManipulatorInfo");
}

TEST(TesseractCommonSerializeUnit, JointState)  // NOLINT
{
  JointState joint_state;
  joint_state.joint_names = { "joint_1", "joint_2", "joint_3" };
  joint_state.position = Eigen::VectorXd::Constant(3, 5);
  joint_state.velocity = Eigen::VectorXd::Constant(3, 6);
  joint_state.acceleration = Eigen::VectorXd::Constant(3, 7);
  joint_state.effort = Eigen::VectorXd::Constant(3, 8);
  joint_state.time = 100;

  tesseract_common::testSerialization<JointState>(joint_state, "JointState");
}

TEST(TesseractCommonSerializeUnit, JointTrajectory)  // NOLINT
{
  JointState joint_state;
  joint_state.joint_names = { "joint_1", "joint_2", "joint_3" };
  joint_state.position = Eigen::VectorXd::Constant(3, 5);
  joint_state.velocity = Eigen::VectorXd::Constant(3, 6);
  joint_state.acceleration = Eigen::VectorXd::Constant(3, 7);
  joint_state.effort = Eigen::VectorXd::Constant(3, 8);
  joint_state.time = 100;

  JointTrajectory trajectory;
  trajectory.states.push_back(joint_state);
  trajectory.description = "this is a test";

  tesseract_common::testSerialization<JointTrajectory>(trajectory, "JointTrajectory");
}

TEST(TesseractCommonSerializeUnit, AllowedCollisionMatrix)  // NOLINT
{
  auto object = std::make_shared<AllowedCollisionMatrix>();
  tesseract_common::testSerialization<AllowedCollisionMatrix>(*object, "EmptyAllowedCollisionMatrix");
  object->addAllowedCollision("link_1", "link2", "reason1");
  object->addAllowedCollision("link_2", "link1", "reason2");
  object->addAllowedCollision("link_4", "link3", "reason3");
  object->addAllowedCollision("link_5", "link2", "reason4");
  tesseract_common::testSerialization<AllowedCollisionMatrix>(*object, "AllowedCollisionMatrix");
}

TEST(TesseractCommonSerializeUnit, CalibrationInfo)  // NOLINT
{
  auto object = std::make_shared<CalibrationInfo>();
  tesseract_common::testSerialization<CalibrationInfo>(*object, "EmptyCalibrationInfo");
  object->joints["test"].setIdentity();
  object->joints["test"].translate(Eigen::Vector3d(2, 4, 8));
  tesseract_common::testSerialization<CalibrationInfo>(*object, "CalibrationInfo");
}

TEST(TesseractCommonSerializeUnit, CollisionMarginData)  // NOLINT
{
  auto object = std::make_shared<CollisionMarginData>();
  tesseract_common::testSerialization<CollisionMarginData>(*object, "EmptyCollisionMarginData");
  object->setPairCollisionMargin("link_1", "link2", 1.1);
  object->setPairCollisionMargin("link_2", "link1", 2.2);
  object->setPairCollisionMargin("link_4", "link3", 3.3);
  object->setPairCollisionMargin("link_5", "link2", -4.4);
  tesseract_common::testSerialization<CollisionMarginData>(*object, "CollisionMarginData");
}

TEST(TesseractCommonSerializeUnit, ContactManagersPluginInfo)  // NOLINT
{
  auto object = std::make_shared<ContactManagersPluginInfo>();
  object->search_paths.insert("path 1");
  object->search_paths.insert("path 2");
  object->search_libraries.insert("search_libraries 1");
  object->search_libraries.insert("search_libraries 2");
  object->search_libraries.insert("search_libraries 3");

  {
    PluginInfoContainer container;
    PluginInfo plugin;
    plugin.class_name = "test_class_name";
    plugin.config["test"] = "value";
    object->discrete_plugin_infos.default_plugin = "test_string";
    object->discrete_plugin_infos.plugins["plugin_key"] = plugin;
  }
  {
    PluginInfoContainer container;
    PluginInfo plugin;
    plugin.class_name = "test_class_name 2";
    plugin.config["test2"] = "value2";
    object->continuous_plugin_infos.default_plugin = "test_string2";
    object->continuous_plugin_infos.plugins["plugin_key2"] = plugin;
  }

  tesseract_common::testSerialization<ContactManagersPluginInfo>(*object, "ContactManagersPluginInfo");
}

TEST(TesseractCommonSerializeUnit, TaskComposerPluginInfo)  // NOLINT
{
  auto object = std::make_shared<TaskComposerPluginInfo>();
  object->search_paths.insert("path 1");
  object->search_paths.insert("path 2");
  object->search_libraries.insert("search_libraries 1");
  object->search_libraries.insert("search_libraries 2");
  object->search_libraries.insert("search_libraries 3");

  {
    PluginInfo plugin;
    plugin.class_name = "test_class_name";
    plugin.config["test"] = "value";
    object->executor_plugin_infos.default_plugin = "test_string";
    object->executor_plugin_infos.plugins["plugin_key"] = plugin;
  }
  {
    PluginInfo plugin;
    plugin.class_name = "test_class_name 2";
    plugin.config["test2"] = "value2";
    object->node_plugin_infos.default_plugin = "test_string2";
    object->node_plugin_infos.plugins["plugin_key2"] = plugin;
  }

  tesseract_common::testSerialization<TaskComposerPluginInfo>(*object, "TaskComposerPluginInfo");
}

TEST(TesseractCommonSerializeUnit, KinematicsPluginInfo)  // NOLINT
{
  auto object = std::make_shared<KinematicsPluginInfo>();
  object->search_paths.insert("path 1");
  object->search_paths.insert("path 2");
  object->search_libraries.insert("search_libraries 1");
  object->search_libraries.insert("search_libraries 2");
  object->search_libraries.insert("search_libraries 3");

  {
    PluginInfo plugin;
    plugin.class_name = "test_class_name";
    plugin.config["test"] = "value";
    object->fwd_plugin_infos["plugin 1"].default_plugin = "test_string";
    object->fwd_plugin_infos["plugin 1"].plugins["plugin_key"] = plugin;
    object->fwd_plugin_infos["plugin 2"].default_plugin = "test_string2";
    object->fwd_plugin_infos["plugin 2"].plugins["plugin_key2"] = plugin;
  }
  {
    PluginInfo plugin;
    plugin.class_name = "test_class_name 2";
    plugin.config["test2"] = "value2";
    object->inv_plugin_infos["inv plugin 1"].default_plugin = "test_string3";
    object->inv_plugin_infos["inv plugin 1"].plugins["plugin_key3"] = plugin;
    object->inv_plugin_infos["inv plugin 2"].default_plugin = "test_string4";
    object->inv_plugin_infos["inv plugin 2"].plugins["plugin_key4"] = plugin;
  }

  tesseract_common::testSerialization<KinematicsPluginInfo>(*object, "KinematicsPluginInfo");
}

TEST(TesseractCommonSerializeUnit, PluginInfo)  // NOLINT
{
  auto object = std::make_shared<PluginInfo>();
  object->class_name = "test_class_name";
  object->config["test"] = M_PI;
  tesseract_common::testSerialization<PluginInfo>(*object, "PluginInfo");
}

TEST(TesseractCommonSerializeUnit, PluginInfoContainer)  // NOLINT
{
  auto object = std::make_shared<PluginInfoContainer>();
  auto plugin = std::make_shared<PluginInfo>();
  plugin->class_name = "test_class_name";
  plugin->config["test"] = "value";
  object->default_plugin = "test_string";
  object->plugins["plugin_key"] = *plugin;
  tesseract_common::testSerialization<PluginInfoContainer>(*object, "PluginInfoContainer");
}

TEST(TesseractCommonSerializeUnit, VectorXd)  // NOLINT
{
  {  // Serialize empty object
    Eigen::VectorXd ev;
    {
      std::ofstream os(tesseract_common::getTempPath() + "eigen_vector_xd_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(ev);
    }

    Eigen::VectorXd nev;
    {
      std::ifstream ifs(tesseract_common::getTempPath() + "eigen_vector_xd_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nev);
    }
  }

  // Serialize to object which already has data
  for (int i = 0; i < 5; ++i)
  {
    Eigen::VectorXd ev = Eigen::VectorXd::Random(6);

    {
      std::ofstream os(tesseract_common::getTempPath() + "eigen_vector_xd_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(ev);
    }

    Eigen::VectorXd nev = Eigen::VectorXd::Random(6);
    {
      std::ifstream ifs(tesseract_common::getTempPath() + "eigen_vector_xd_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nev);
    }

    EXPECT_TRUE(ev.isApprox(nev, 1e-5));
  }

  // Serialize to object which already has data and different size
  for (int i = 0; i < 5; ++i)
  {
    Eigen::VectorXd ev = Eigen::VectorXd::Random(6);

    {
      std::ofstream os(tesseract_common::getTempPath() + "eigen_vector_xd_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(ev);
    }

    Eigen::VectorXd nev = Eigen::VectorXd::Random(3);
    {
      std::ifstream ifs(tesseract_common::getTempPath() + "eigen_vector_xd_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nev);
    }

    EXPECT_TRUE(ev.isApprox(nev, 1e-5));
  }

  // Default use case
  for (int i = 0; i < 5; ++i)
  {
    Eigen::VectorXd ev = Eigen::VectorXd::Random(6);

    {
      std::ofstream os(tesseract_common::getTempPath() + "eigen_vector_xd_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(ev);
    }

    Eigen::VectorXd nev;
    {
      std::ifstream ifs(tesseract_common::getTempPath() + "eigen_vector_xd_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nev);
    }

    EXPECT_TRUE(ev.isApprox(nev, 1e-5));
  }
}

TEST(TesseractCommonSerializeUnit, MatrixX2d)  // NOLINT
{
  {  // Serialize empty
    Eigen::MatrixX2d em;

    {
      std::ofstream os(tesseract_common::getTempPath() + "eigen_matrix_x2d_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(em);
    }

    Eigen::MatrixX2d nem;
    {
      std::ifstream ifs(tesseract_common::getTempPath() + "eigen_matrix_x2d_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nem);
    }

    EXPECT_TRUE(em.isApprox(nem, 1e-5));
  }

  // Serialize to object which already has data
  for (int i = 0; i < 5; ++i)
  {
    Eigen::MatrixX2d em = Eigen::MatrixX2d::Random(4, 2);

    {
      std::ofstream os(tesseract_common::getTempPath() + "eigen_matrix_x2d_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(em);
    }

    Eigen::MatrixX2d nem = Eigen::MatrixX2d::Random(4, 2);
    {
      std::ifstream ifs(tesseract_common::getTempPath() + "eigen_matrix_x2d_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nem);
    }

    EXPECT_TRUE(em.isApprox(nem, 1e-5));
  }

  // Serialize to object which already has data and different size
  for (int i = 0; i < 5; ++i)
  {
    Eigen::MatrixX2d em = Eigen::MatrixX2d::Random(4, 2);

    {
      std::ofstream os(tesseract_common::getTempPath() + "eigen_matrix_x2d_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(em);
    }

    Eigen::MatrixX2d nem = Eigen::MatrixX2d::Random(2, 2);
    {
      std::ifstream ifs(tesseract_common::getTempPath() + "eigen_matrix_x2d_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nem);
    }

    EXPECT_TRUE(em.isApprox(nem, 1e-5));
  }

  // Default
  for (int i = 0; i < 5; ++i)
  {
    Eigen::MatrixX2d em = Eigen::MatrixX2d::Random(4, 2);

    {
      std::ofstream os(tesseract_common::getTempPath() + "eigen_matrix_x2d_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(em);
    }

    Eigen::MatrixX2d nem;
    {
      std::ifstream ifs(tesseract_common::getTempPath() + "eigen_matrix_x2d_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(nem);
    }

    EXPECT_TRUE(em.isApprox(nem, 1e-5));
  }
}

TEST(TesseractCommonSerializeUnit, Isometry3d)  // NOLINT
{
  for (int i = 0; i < 5; ++i)
  {
    Eigen::Isometry3d pose =
        Eigen::Isometry3d::Identity() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::Random().normalized());
    pose.translation() = Eigen::Vector3d::Random();

    {
      std::ofstream os(tesseract_common::getTempPath() + "eigen_isometry3d_boost.xml");
      boost::archive::xml_oarchive oa(os);
      oa << BOOST_SERIALIZATION_NVP(pose);
    }

    Eigen::Isometry3d npose;
    {
      std::ifstream ifs(tesseract_common::getTempPath() + "eigen_isometry3d_boost.xml");
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);

      // restore the schedule from the archive
      ia >> BOOST_SERIALIZATION_NVP(npose);
    }

    EXPECT_TRUE(pose.isApprox(npose, 1e-5));
  }
}

/** @brief Atomic do not have a copy constructor so must have implement one for your class */
struct TestAtomic
{
  std::atomic<bool> value{ true };

  TestAtomic() = default;
  ~TestAtomic() = default;
  TestAtomic(const TestAtomic& other) { *this = other; }
  TestAtomic& operator=(const TestAtomic& other)
  {
    value = other.value.load();
    return *this;
  }
  TestAtomic(TestAtomic&& other) noexcept : value(other.value.load()) {}
  TestAtomic& operator=(TestAtomic&& other) noexcept
  {
    value = other.value.load();
    return *this;
  }

  bool operator==(const TestAtomic& rhs) const { return rhs.value.load() == value.load(); }
  bool operator!=(const TestAtomic& rhs) const { return rhs.value.load() != value.load(); }

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
  {
    ar& BOOST_SERIALIZATION_NVP(value);
  }
};

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(TestAtomic)
BOOST_CLASS_EXPORT_IMPLEMENT(TestAtomic)

TEST(TesseractCommonSerializeUnit, StdAtomic)  // NOLINT
{
  TestAtomic object;
  object.value = true;
  tesseract_common::testSerialization<TestAtomic>(object, "TestAtomic");
}

struct ExtensionMacroTestA
{
  double a{ 0 };
};

TESSERACT_CLASS_EXTENSION(ExtensionMacroTestA, "etax", "etab")

struct ExtensionMacroTestB
{
  double b{ 0 };
};

TEST(TesseractCommonSerializeUnit, ExtensionXmlMacro)  // NOLINT
{
  std::string ext = tesseract_common::serialization::xml::extension<ExtensionMacroTestA>::value;
  EXPECT_EQ(ext, "etax");

  std::string default_ext = tesseract_common::serialization::xml::extension<ExtensionMacroTestB>::value;
  EXPECT_EQ(default_ext, "trsx");
}

TEST(TesseractCommonSerializeUnit, ExtensionBinaryMacro)  // NOLINT
{
  std::string ext = tesseract_common::serialization::binary::extension<ExtensionMacroTestA>::value;
  EXPECT_EQ(ext, "etab");

  std::string default_ext = tesseract_common::serialization::binary::extension<ExtensionMacroTestB>::value;
  EXPECT_EQ(default_ext, "trsb");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
