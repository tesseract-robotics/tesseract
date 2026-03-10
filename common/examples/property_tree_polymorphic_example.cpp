/**
 * @page polymorphic_property_tree_example Tesseract PropertyTree Polymorphic Types Example
 *
 * @section polymorphic_property_tree_overview Overview
 *
 * This example demonstrates the polymorphic type system in Tesseract's PropertyTree,
 * which enables schema validation for type hierarchies where fields accept a base type
 * or any registered derived type.
 *
 * Polymorphic types are useful for:
 * - **Type Inheritance**: Accept base class or any subclass
 * - **Plugin Architectures**: Accept base plugin type or derived implementations
 * - **Constraint Systems**: Accept base constraint or specific constraint types
 * - **Extensibility**: New derived types can be registered without schema changes
 * - **Dynamic Dispatch**: Configuration data includes "type" field for validation selection
 *
 * @section polymorphic_property_tree_concepts Key Concepts
 *
 * - **Base Type**: A registered type that other types can derive from
 * - **Derived Type**: A type that extends the base type and is registered as such
 * - **Type Registration**: Using TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE macro
 * - **Type Field**: A required field in configuration specifying the concrete type
 * - **Polymorphic Validation**: Field marked with acceptsDerivedTypes() accepts any registered derived type
 * - **Type Safety**: Validation checks if the type is valid in the hierarchy
 *
 * @section polymorphic_property_tree_workflow Polymorphic Validation Workflow
 *
 * The example demonstrates:
 *
 * 1. **Register Base Schema**: Define the base type schema
 * 2. **Register Derived Schemas**: Define schemas for each derived type
 * 3. **Register Inheritance**: Tell registry which types derive from base
 * 4. **Mark Field**: Use acceptsDerivedTypes() on fields accepting polymorphic types
 * 5. **Validate**: System checks "type" field is valid and validates against corresponding schema
 *
 * @section polymorphic_property_tree_code Example Code
 *
 * @snippet property_tree_polymorphic_example.cpp type_registration_start
 *
 * First, register the type hierarchy. Base types and derived types each get schemas.
 * Then use TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE to establish relationships.
 *
 * @snippet property_tree_polymorphic_example.cpp field_definition_start
 *
 * Mark sequence/map fields with acceptsDerivedTypes() to allow derived types.
 * The validator will check each element's "type" field against the hierarchy.
 *
 * @snippet property_tree_polymorphic_example.cpp polymorphic_validation_start
 *
 * During validation, the system checks if each element's type is registered as
 * derived from (or equal to) the base type, then validates against that type's schema.
 *
 * @section polymorphic_property_tree_comparison Comparing Polymorphism Approaches
 *
 * PropertyTree offers two polymorphism mechanisms:
 *
 * | Feature | Derived Types | OneOf |
 * |---------|---|---|
 * | **Selection Method** | Type field (explicit naming) | Required field match (implicit) |
 * | **Type Discovery** | "type": "DerivedClassName" | Keys present in config determine branch |
 * | **Registration** | Runtime via macros (compile-time execution) | Static in schema definition |
 * | **Extensibility** | Very extensible - new types can be added | Requires schema modification |
 * | **Type Hierarchies** | Supports inheritance chains | Not designed for hierarchies |
 * | **Error Messages** | Clear type name in error | Shows expected/actual keys |
 * | **Typical Use Case** | Plugins, constraints, different algorithm implementations | Different communication methods,
 * shape types |
 *
 * @section polymorphic_property_tree_best_practices Best Practices
 *
 * 1. **Always Include Type Field**: Polymorphic fields need explicit type information
 * 2. **Use Clear Type Names**: Make type names match class names when possible
 * 3. **Register Early**: Use TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE at module init time
 * 4. **Document Type Hierarchy**: Add doc attributes explaining what types are valid
 * 5. **Provide Type Defaults**: Consider if a default type makes sense for the field
 * 6. **Validate Complete Config**: Always call validate() and check all error messages
 * 7. **Consider Schema Versioning**: Plan for adding new derived types in the future
 * 8. **Test All Types**: Write validation tests covering each derived type variant
 *
 * @section polymorphic_property_tree_run Running the Example
 *
 * Execute the example:
 * @code
 * ./tesseract_common_polymorphic_property_tree_example
 * @endcode
 *
 * The example demonstrates:
 * - Registering type hierarchies
 * - Validating polymorphic sequences with multiple types
 * - Error handling for invalid types
 * - Accessing derived type instances
 * - Schema inspection with metadata
 */

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <tesseract/common/property_tree.h>
#include <tesseract/common/schema_registry.h>

using namespace tesseract::common;
using namespace tesseract::common::property_attribute;
using namespace tesseract::common::property_type;

int main(int /*argc*/, char** /*argv*/)
{
  std::cout << "========================================================\n";
  std::cout << "  PropertyTree Polymorphic Types (Derived Type) Example \n";
  std::cout << "========================================================\n\n";

  // Get registry instance first for use in lambda captures
  auto registry = SchemaRegistry::instance();

  // ====================================================================
  // Setup: Register Type Hierarchies
  // ====================================================================
  std::cout << "Step 1: Registering Type Hierarchies\n";
  std::cout << "------------------------------------\n";

  //! [type_registration_start]
  // clang-format off
  // Define base constraint schema with common fields
  auto base_constraint_schema = PropertyTreeBuilder()
      .attribute(TYPE, CONTAINER)
      .doubleNum("weight").defaultVal(1.0).minimum(0.0).doc("Constraint weight in optimization").done()
      .build();

  // Define position constraint schema (extends base)
  auto position_constraint_schema = PropertyTreeBuilder()
      .attribute(TYPE, CONTAINER)
      .doubleNum("weight").defaultVal(1.0).minimum(0.0).doc("Constraint weight in optimization").done()
      .doubleNum("target_x").required().doc("Target position X coordinate").done()
      .doubleNum("target_y").required().doc("Target position Y coordinate").done()
      .doubleNum("target_z").required().doc("Target position Z coordinate").done()
      .doubleNum("target_tolerance").defaultVal(0.01).minimum(0.0).doc("Position tolerance in meters").done()
      .build();

  // Define orientation constraint schema (extends base)
  auto orientation_constraint_schema = PropertyTreeBuilder()
      .attribute(TYPE, CONTAINER)
      .doubleNum("weight").defaultVal(1.0).minimum(0.0).doc("Constraint weight in optimization").done()
      .doubleNum("target_roll").required().doc("Target roll angle in radians").done()
      .doubleNum("target_pitch").required().doc("Target pitch angle in radians").done()
      .doubleNum("target_yaw").required().doc("Target yaw angle in radians").done()
      .doubleNum("target_tolerance").defaultVal(0.1).minimum(0.0).doc("Orientation tolerance in radians").done()
      .build();
  //clang-format on

  // Register all schemas
  registry->registerSchema("Constraint", base_constraint_schema);
  registry->registerSchema("PositionConstraint", position_constraint_schema);
  registry->registerSchema("OrientationConstraint", orientation_constraint_schema);

  // Register inheritance relationships
  registry->registerDerivedType("Constraint", "PositionConstraint");
  registry->registerDerivedType("Constraint", "OrientationConstraint");
  //! [type_registration_start]

  // Verify registration
  auto all_types = registry->getDerivedTypes("Constraint");
  std::cout << "Registered types that derive from 'Constraint':\n";
  for (const auto& t : all_types)
  {
    std::cout << "  - " << t << "\n";
  }
  std::cout << "\n";

  // ====================================================================
  // Example 1: Define Field Accepting Polymorphic Types
  // ====================================================================
  std::cout << "Step 2: Define Field Accepting Polymorphic Types\n";
  std::cout << "-----------------------------------------------\n";

  // clang-format off
  //! [field_definition_start]
  // Create a trajectory schema with a container of polymorphic constraints
  auto trajectory_schema = PropertyTreeBuilder()
      .attribute(TYPE, CONTAINER)
      .string("name").required().doc("Name of this trajectory").done()
      .container("constraint").doc("Task constraint (any constraint type can be used)")
        .customType("constraint", "Constraint")
        .acceptsDerivedTypes()
        .validator(validateCustomType)
      .done()
      .build();
  //! [field_definition_start]
  // clang-format on

  std::cout << "Created trajectory schema with polymorphic constraint field\n\n";

  // ====================================================================
  // Example 1: Valid Configuration with Position Constraint
  // ====================================================================
  std::cout << "Example 1: Valid Position Constraint\n";
  std::cout << "-----------------------------------\n";

  //! [polymorphic_validation_start]
  YAML::Node valid_config;
  valid_config["name"] = "approach_trajectory";
  valid_config["constraint"]["type"] = "PositionConstraint";
  valid_config["constraint"]["weight"] = 2.0;
  valid_config["constraint"]["target_x"] = 1.0;
  valid_config["constraint"]["target_y"] = 2.0;
  valid_config["constraint"]["target_z"] = 3.0;
  valid_config["constraint"]["target_tolerance"] = 0.005;

  auto schema_copy = trajectory_schema;
  schema_copy.mergeConfig(valid_config);
  auto errors = schema_copy.validate();
  //! [polymorphic_validation_start]

  if (errors.empty())
  {
    std::cout << "✓ Validation passed!\n";
    auto name_value = schema_copy.at("name").template as<std::string>();
    std::cout << "  Trajectory: " << name_value << "\n";
    const auto& constraint_node = schema_copy.at("constraint");
    auto constraint_type = constraint_node.at("type").template as<std::string>();
    auto weight = constraint_node.at("weight").template as<double>();
    std::cout << "  Constraint type: " << constraint_type << " (weight: " << weight << ")\n";
  }
  else
  {
    std::cout << "✗ Validation failed with " << errors.size() << " errors:\n";
    for (const auto& err : errors)
      std::cout << "  - " << err << "\n";
  }
  std::cout << "\n";

  // ====================================================================
  // Example 2: Valid Configuration with Orientation Constraint
  // ====================================================================
  std::cout << "Example 2: Valid Orientation Constraint\n";
  std::cout << "--------------------------------------\n";

  YAML::Node orient_config;
  orient_config["name"] = "reorientation_trajectory";
  orient_config["constraint"]["type"] = "OrientationConstraint";
  orient_config["constraint"]["weight"] = 1.5;
  orient_config["constraint"]["target_roll"] = 0.0;
  orient_config["constraint"]["target_pitch"] = 1.5708;  // 90 degrees
  orient_config["constraint"]["target_yaw"] = 0.0;
  orient_config["constraint"]["target_tolerance"] = 0.1;

  schema_copy = trajectory_schema;
  schema_copy.mergeConfig(orient_config);
  errors = schema_copy.validate();

  if (errors.empty())
  {
    std::cout << "✓ Validation passed!\n";
    auto name_value = schema_copy.at("name").template as<std::string>();
    std::cout << "  Trajectory: " << name_value << "\n";
    const auto& constraint_node = schema_copy.at("constraint");
    auto constraint_type = constraint_node.at("type").template as<std::string>();
    auto weight = constraint_node.at("weight").template as<double>();
    std::cout << "  Constraint type: " << constraint_type << " (weight: " << weight << ")\n";
  }
  else
  {
    std::cout << "✗ Validation failed with " << errors.size() << " errors:\n";
    for (const auto& err : errors)
      std::cout << "  - " << err << "\n";
  }
  std::cout << "\n";

  // ====================================================================
  // Example 3: Invalid Type
  // ====================================================================
  std::cout << "Example 3: Invalid Constraint Type\n";
  std::cout << "---------------------------\n";

  YAML::Node invalid_type_config;
  invalid_type_config["name"] = "bad_trajectory";
  invalid_type_config["constraint"]["type"] = "UnknownConstraint";  // Invalid!
  invalid_type_config["constraint"]["weight"] = 1.0;

  schema_copy = trajectory_schema;
  schema_copy.mergeConfig(invalid_type_config);
  errors = schema_copy.validate();

  if (!errors.empty())
  {
    std::cout << "✗ Validation found " << errors.size() << " error(s):\n";
    for (const auto& err : errors)
      std::cout << "  - " << err << "\n";
  }
  std::cout << "\n";

  // ====================================================================
  // Example 4: Missing Required Field for Derived Type
  // ====================================================================
  std::cout << "Example 4: Missing Required Field (PositionConstraint)\n";
  std::cout << "------------------------------------------------------\n";

  YAML::Node missing_field_config;
  missing_field_config["name"] = "incomplete_trajectory";
  missing_field_config["constraint"]["type"] = "PositionConstraint";
  missing_field_config["constraint"]["weight"] = 1.0;
  // Missing: target_x, target_y, target_z, target_tolerance

  schema_copy = trajectory_schema;
  schema_copy.mergeConfig(missing_field_config);
  errors = schema_copy.validate();

  if (!errors.empty())
  {
    std::cout << "✗ Validation found " << errors.size() << " error(s):\n";
    for (const auto& err : errors)
      std::cout << "  - " << err << "\n";
  }
  std::cout << "\n";

  // ====================================================================
  // Example 5: Type Compatibility Check
  // ====================================================================
  std::cout << "Example 5: Type Compatibility Checks\n";
  std::cout << "-----------------------------------\n";

  bool is_pos_derived = registry->isDerivedFrom("Constraint", "PositionConstraint");
  bool is_orient_derived = registry->isDerivedFrom("Constraint", "OrientationConstraint");
  bool is_constraint_derived = registry->isDerivedFrom("Constraint", "Constraint");
  bool is_unrelated = registry->isDerivedFrom("Constraint", "SomeOtherType");

  std::cout << R"(isDerivedFrom("Constraint", "PositionConstraint") = )" << (is_pos_derived ? "true" : "false") << "\n";
  std::cout << R"(isDerivedFrom("Constraint", "OrientationConstraint") = )" << (is_orient_derived ? "true" : "false")
            << "\n";
  std::cout << R"(isDerivedFrom("Constraint", "Constraint") = )" << (is_constraint_derived ? "true" : "false") << "\n";
  std::cout << R"(isDerivedFrom("Constraint", "SomeOtherType") = )" << (is_unrelated ? "true" : "false") << "\n";
  std::cout << "\n";

  // ====================================================================
  // Example 6: Schema Inspection
  // ====================================================================
  std::cout << "Example 6: Schema Inspection\n";
  std::cout << "----------------------------\n";

  const auto& pos_constraint = registry->get("PositionConstraint");
  auto tolerance_attr = pos_constraint.at("target_tolerance").getAttribute(DOC);

  std::cout << "PositionConstraint schema attributes:\n";
  std::cout << "  Field: target_tolerance\n";
  if (tolerance_attr)
  {
    std::cout << "    Doc: " << tolerance_attr->template as<std::string>() << "\n";
  }
  std::cout << "\n";

  std::cout << "========================================================\n";
  std::cout << "     Example Complete - Polymorphic Types Supported      \n";
  std::cout << "========================================================\n";

  return 0;
}
