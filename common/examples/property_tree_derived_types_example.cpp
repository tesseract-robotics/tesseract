/**
 * @page derived_types_example Derived Types with Plugin Info Structure Example
 *
 * @brief Example demonstrating derived types support with the standard plugin info structure
 *
 * This example shows how to:
 * 1. Register base and derived types in the schema registry
 * 2. Create schemas that accept derived types using `acceptsDerivedTypes()`
 * 3. Validate YAML configurations using the plugin info structure (class + config)
 * 4. Handle single entries, arrays, and maps of heterogeneous derived types
 * 5. Understand error handling for invalid types and configurations
 *
 * The plugin info structure pattern is ideal for plugin architectures and extensible
 * type systems where configuration needs to specify both the concrete type (via "class"
 * field) and its type-specific parameters (via "config" field).
 */

#include <tesseract/common/property_tree.h>
#include <tesseract/common/schema_registration.h>
#include <tesseract/common/schema_registry.h>
#include <iostream>

using namespace tesseract::common;

// ============================================================================
// Step 1: Define and Register Schemas
// ============================================================================

// Base Constraint Schema
PropertyTree createBaseConstraintSchema()
{
  return PropertyTreeBuilder().container("base_constraint").doc("Base constraint properties").done().build();
}

// Derived Type 1: JointPositionConstraint
PropertyTree createJointPositionConstraintSchema()
{
  return PropertyTreeBuilder()
      .container("joint_position_constraint")
      .doc("Constrains a joint to a specific position")
      .string("joint")
      .doc("Name of the joint")
      .required()
      .done()
      .doubleNum("position")
      .doc("Target position value")
      .required()
      .done()
      .done()
      .build();
}

// Derived Type 2: CartesianVelocityConstraint
PropertyTree createCartesianVelocityConstraintSchema()
{
  return PropertyTreeBuilder()
      .container("cartesian_velocity_constraint")
      .doc("Limits the Cartesian velocity of the end-effector")
      .string("frame")
      .doc("Reference frame")
      .required()
      .done()
      .doubleNum("max_velocity")
      .doc("Maximum allowed velocity")
      .required()
      .minimum(0.0)
      .done()
      .done()
      .build();
}

// Derived Type 3: CollisionConstraint
PropertyTree createCollisionConstraintSchema()
{
  return PropertyTreeBuilder()
      .container("collision_constraint")
      .doc("Enforces collision avoidance")
      .string("constraint_type")
      .doc("Type of constraint: USE_LIMITS or PENALTY")
      .required()
      .enumValues({ "USE_LIMITS", "PENALTY" })
      .done()
      .doubleNum("safety_margin")
      .doc("Minimum distance to maintain")
      .defaultVal(0.05)
      .done()
      .done()
      .build();
}

// ============================================================================
// Step 2: Example 1 - Single Constraint Entry
// ============================================================================

void example_single_entry()
{
  std::cout << "\n=== Example 1: Single Constraint Entry ===\n";

  // Create schema that accepts derived constraint types
  PropertyTree schema = PropertyTreeBuilder()
                            .container("constraint_config")
                            .doc("Constraint configuration accepting derived types")
                            .customType("constraint", "BaseConstraint")
                            .doc("A constraint (can be any registered BaseConstraint derived type)")
                            .acceptsDerivedTypes()
                            .done()
                            .done()
                            .build();

  // Valid YAML configuration using plugin info structure
  YAML::Node config = YAML::Load(R"(
constraint_config:
  constraint:
    class: JointPositionConstraint
    config:
      joint: joint_1
      position: 0.785
)");

  PropertyTree pt = PropertyTree::fromYAML(config);
  pt.mergeConfig(config);

  // Validate
  std::vector<std::string> errors = pt.validate();
  if (errors.empty())
  {
    std::cout << "✓ Validation passed for single constraint entry\n";
  }
  else
  {
    std::cout << "✗ Validation failed:\n";
    for (const auto& error : errors)
      std::cout << "  - " << error << "\n";
  }
}

// ============================================================================
// Step 3: Example 2 - Array of Constraints
// ============================================================================

void example_constraint_array()
{
  std::cout << "\n=== Example 2: Array of Constraints ===\n";

  // Create schema that accepts an array of derived constraint types
  PropertyTree schema = PropertyTreeBuilder()
                            .container("planning_problem")
                            .doc("Planning problem with multiple constraints")
                            .customType("constraints", "List[BaseConstraint]")
                            .doc("Array of constraints (each can be any derived BaseConstraint type)")
                            .acceptsDerivedTypes()
                            .done()
                            .done()
                            .build();

  // Valid YAML with multiple different constraint types
  YAML::Node config = YAML::Load(R"(
planning_problem:
  constraints:
    - class: JointPositionConstraint
      config:
        joint: joint_1
        position: 0.0
    - class: CartesianVelocityConstraint
      config:
        frame: tool0
        max_velocity: 1.5
    - class: CollisionConstraint
      config:
        constraint_type: USE_LIMITS
        safety_margin: 0.1
)");

  PropertyTree pt = PropertyTree::fromYAML(config);
  pt.mergeConfig(config);

  // Validate
  std::vector<std::string> errors = pt.validate();
  if (errors.empty())
  {
    std::cout << "✓ Validation passed for constraint array\n";
    std::cout << "  Validated " << config["planning_problem"]["constraints"].size() << " constraints\n";
  }
  else
  {
    std::cout << "✗ Validation failed:\n";
    for (const auto& error : errors)
      std::cout << "  - " << error << "\n";
  }
}

// ============================================================================
// Step 4: Example 3 - Map of Named Constraints
// ============================================================================

void example_constraint_map()
{
  std::cout << "\n=== Example 3: Map of Named Constraints ===\n";

  // Create schema that accepts a map of named derived constraint types
  PropertyTree schema = PropertyTreeBuilder()
                            .container("scenario")
                            .doc("Scenario with named constraints")
                            .customType("constraints", "Map[String,BaseConstraint]")
                            .doc("Map of constraint names to constraint definitions")
                            .acceptsDerivedTypes()
                            .done()
                            .done()
                            .build();

  // Valid YAML with named constraints
  YAML::Node config = YAML::Load(R"(
scenario:
  constraints:
    home_position:
      class: JointPositionConstraint
      config:
        joint: wrist_3
        position: 1.57
    safety_limits:
      class: CollisionConstraint
      config:
        constraint_type: USE_LIMITS
        safety_margin: 0.05
    smooth_motion:
      class: CartesianVelocityConstraint
      config:
        frame: workspace_center
        max_velocity: 2.0
)");

  PropertyTree pt = PropertyTree::fromYAML(config);
  pt.mergeConfig(config);

  // Validate
  std::vector<std::string> errors = pt.validate();
  if (errors.empty())
  {
    std::cout << "✓ Validation passed for constraint map\n";
    std::cout << "  Validated " << config["scenario"]["constraints"].size() << " named constraints:\n";
    for (auto it = config["scenario"]["constraints"].begin(); it != config["scenario"]["constraints"].end(); ++it)
    {
      std::cout << "    - " << it->first.as<std::string>() << ": " << it->second["class"].as<std::string>() << "\n";
    }
  }
  else
  {
    std::cout << "✗ Validation failed:\n";
    for (const auto& error : errors)
      std::cout << "  - " << error << "\n";
  }
}

// ============================================================================
// Step 5: Example 4 - Error Handling
// ============================================================================

void example_error_handling()
{
  std::cout << "\n=== Example 4: Error Handling ===\n";

  PropertyTree schema = PropertyTreeBuilder()
                            .container("problem")
                            .customType("constraint", "BaseConstraint")
                            .acceptsDerivedTypes()
                            .done()
                            .done()
                            .build();

  // Example 4a: Missing 'class' field
  {
    std::cout << "\nTest 4a: Missing 'class' field\n";
    YAML::Node config = YAML::Load(R"(
problem:
  constraint:
    config:
      joint: joint_1
)");  // Missing "class" field

    PropertyTree pt = PropertyTree::fromYAML(config);
    pt.mergeConfig(config);
    std::vector<std::string> errors = pt.validate();

    if (!errors.empty())
    {
      std::cout << "✓ Correctly caught error:\n";
      for (const auto& error : errors)
        std::cout << "  " << error << "\n";
    }
  }

  // Example 4b: Invalid derived type
  {
    std::cout << "\nTest 4b: Invalid derived type\n";
    YAML::Node config = YAML::Load(R"(
problem:
  constraint:
    class: NonExistentConstraint
    config:
      param: value
)");

    PropertyTree pt = PropertyTree::fromYAML(config);
    pt.mergeConfig(config);
    std::vector<std::string> errors = pt.validate();

    if (!errors.empty())
    {
      std::cout << "✓ Correctly caught error:\n";
      for (const auto& error : errors)
        std::cout << "  " << error << "\n";
    }
  }

  // Example 4c: Invalid config value
  {
    std::cout << "\nTest 4c: Invalid config (missing required field)\n";
    YAML::Node config = YAML::Load(R"(
problem:
  constraint:
    class: JointPositionConstraint
    config:
      joint: joint_1
)");  // Missing required "position" field

    PropertyTree pt = PropertyTree::fromYAML(config);
    pt.mergeConfig(config);
    std::vector<std::string> errors = pt.validate();

    if (!errors.empty())
    {
      std::cout << "✓ Correctly caught error:\n";
      for (const auto& error : errors)
        std::cout << "  " << error << "\n";
    }
  }
}

// ============================================================================
// Main - Registration and Execution
// ============================================================================

int main()
{
  std::cout << "Derived Types Support Examples\n";
  std::cout << "==============================\n";

  // Register the base constraint type and derived types
  {
    auto registry = SchemaRegistry::instance();

    registry->registerSchema("BaseConstraint", createBaseConstraintSchema());
    registry->registerSchema("JointPositionConstraint", createJointPositionConstraintSchema());
    registry->registerSchema("CartesianVelocityConstraint", createCartesianVelocityConstraintSchema());
    registry->registerSchema("CollisionConstraint", createCollisionConstraintSchema());

    // Register type relationships
    registry->registerDerivedType("BaseConstraint", "JointPositionConstraint");
    registry->registerDerivedType("BaseConstraint", "CartesianVelocityConstraint");
    registry->registerDerivedType("BaseConstraint", "CollisionConstraint");
  }

  // Run examples
  example_single_entry();
  example_constraint_array();
  example_constraint_map();
  example_error_handling();

  std::cout << "\n==============================\n";
  std::cout << "All examples completed!\n";

  return 0;
}
