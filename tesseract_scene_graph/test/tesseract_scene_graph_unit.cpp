#include <gtest/gtest.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/graph_utils.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/filtered_graph.hpp>
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
  SceneGraph g;

  tesseract::LinkPtr link_1(new tesseract::Link("link_1"));
  tesseract::LinkPtr link_2(new tesseract::Link("link_2"));
  tesseract::LinkPtr link_3(new tesseract::Link("link_3"));
  tesseract::LinkPtr link_4(new tesseract::Link("link_4"));
  tesseract::LinkPtr link_5(new tesseract::Link("link_5"));

  g.addLink(link_1);
  g.addLink(link_2);
  g.addLink(link_3);
  g.addLink(link_4);
  g.addLink(link_5);

  tesseract::JointPtr joint_1(new tesseract::Joint("joint_1"));
  joint_1->parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1->parent_link_name = "link_1";
  joint_1->child_link_name = "link_2";
  joint_1->type = tesseract::JointType::FIXED;
  g.addJoint(joint_1);

  tesseract::JointPtr joint_2(new tesseract::Joint("joint_2"));
  joint_2->parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2->parent_link_name = "link_2";
  joint_2->child_link_name = "link_3";
  joint_2->type = tesseract::JointType::PLANAR;
  g.addJoint(joint_2);

  tesseract::JointPtr joint_3(new tesseract::Joint("joint_3"));
  joint_3->parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3->parent_link_name = "link_3";
  joint_3->child_link_name = "link_4";
  joint_3->type = tesseract::JointType::FLOATING;
  g.addJoint(joint_3);

  tesseract::JointPtr joint_4(new tesseract::Joint("joint_4"));
  joint_4->parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4->parent_link_name = "link_2";
  joint_4->child_link_name = "link_5";
  joint_4->type = tesseract::JointType::REVOLUTE;
  g.addJoint(joint_4);

  // Save Graph
  g.saveDOT("/tmp/graph_acyclic_tree_example.dot");

  // Test if the graph is Acyclic
  std::cout << "Is Acyclic: " << g.isAcyclic() << std::endl;

  // Test if the graph is Tree
  std::cout << "Is Tree: " << g.isTree() << std::endl;

  // Test for unused links
  tesseract::LinkPtr link_6(new tesseract::Link("link_6"));
  g.addLink(link_6);
  std::cout << "Free Link, Is Tree: " << g.isTree() << std::endl;
  g.removeLink(link_6->getName());
  std::cout << "Free Link Removed, Is Tree: " << g.isTree() << std::endl;

  tesseract::JointPtr joint_5(new tesseract::Joint("joint_5"));
  joint_5->parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5->parent_link_name = "link_5";
  joint_5->child_link_name = "link_4";
  joint_5->type = tesseract::JointType::CONTINUOUS;
  g.addJoint(joint_5);

  // Save Graph
  g.saveDOT("/tmp/graph_acyclic_not_tree_example.dot");

  // Test if the graph is Acyclic
  std::cout << "Is Acyclic: " << g.isAcyclic() << std::endl;

  // Test if the graph is Tree
  std::cout << "Is Tree: " << g.isTree() << std::endl;

  tesseract::JointPtr joint_6(new tesseract::Joint("joint_6"));
  joint_6->parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_6->parent_link_name = "link_5";
  joint_6->child_link_name = "link_1";
  joint_6->type = tesseract::JointType::CONTINUOUS;
  g.addJoint(joint_6);

  // Save Graph
  g.saveDOT("/tmp/graph_cyclic_not_tree_example.dot");

  // Test if the graph is Acyclic
  std::cout << "Is Acyclic: " << g.isAcyclic() << std::endl;

  // Test if the graph is Tree
  std::cout << "Is Tree: " << g.isTree() << std::endl;

  // Get Shortest Path
  tesseract::graph::SceneGraph::Path path = g.getShortestPath("link_1", "link_4");

// Todo:: Look at using filtered graph for chains and subgraphs
// boost::filtered_graph<

  std::cout << path << std::endl;

  std::cout << (g.getName().c_str()) << std::endl;
}
