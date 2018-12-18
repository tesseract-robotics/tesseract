#include <gtest/gtest.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/graph_utils.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <iostream>
#include <fstream>

//TEST(TesseractBuilderUnit, AddLinksUnit)
//{
//  tesseract::AdjacencyList();
////  tesseract::tesseract_bullet::BulletCastBVHManager checker;
////  addCollisionObjects(checker);
////  runTest(checker);
//}

//int main(int argc, char** argv)
//{
//  testing::InitGoogleTest(&argc, argv);

//  return RUN_ALL_TESTS();
//}

int main(int argc, char** argv)
{
  using namespace tesseract::graph;
  Graph g;

  tesseract::LinkPtr link_1(new tesseract::Link("link_1"));
  tesseract::LinkPtr link_2(new tesseract::Link("link_2"));
  tesseract::LinkPtr link_3(new tesseract::Link("link_3"));
  tesseract::LinkPtr link_4(new tesseract::Link("link_4"));
  tesseract::LinkPtr link_5(new tesseract::Link("link_5"));

  addLink(link_1, g);
  addLink(link_2, g);
  addLink(link_3, g);
  addLink(link_4, g);
  addLink(link_5, g);

  tesseract::JointPtr joint_1(new tesseract::Joint("joint_1"));
  joint_1->parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1->parent_link_name = "link_1";
  joint_1->child_link_name = "link_2";
  joint_1->type = tesseract::JointType::FIXED;
  addJoint(joint_1, g);

  tesseract::JointPtr joint_2(new tesseract::Joint("joint_2"));
  joint_2->parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2->parent_link_name = "link_2";
  joint_2->child_link_name = "link_3";
  joint_2->type = tesseract::JointType::PLANAR;
  addJoint(joint_2, g);

  tesseract::JointPtr joint_3(new tesseract::Joint("joint_3"));
  joint_3->parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3->parent_link_name = "link_3";
  joint_3->child_link_name = "link_4";
  joint_3->type = tesseract::JointType::FLOATING;
  addJoint(joint_3, g);

  tesseract::JointPtr joint_4(new tesseract::Joint("joint_4"));
  joint_4->parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4->parent_link_name = "link_2";
  joint_4->child_link_name = "link_5";
  joint_4->type = tesseract::JointType::REVOLUTE;
  addJoint(joint_4, g);

  // Save Graph
  utils::saveDOT("/tmp/graph_acyclic_example.dot", g);

  tesseract::JointPtr joint_5(new tesseract::Joint("joint_5"));
  joint_5->parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5->parent_link_name = "link_5";
  joint_5->child_link_name = "link_4";
  joint_5->type = tesseract::JointType::CONTINUOUS;
  addJoint(joint_5, g);

  // Save Graph
  utils::saveDOT("/tmp/graph_example.dot", g);

  // Get Shortest Path
  auto path = utils::getShortestPath("link_1", "link_4", g);

  std::cout << path;

  std::cout << (getName(g).c_str()) << std::endl;
}
