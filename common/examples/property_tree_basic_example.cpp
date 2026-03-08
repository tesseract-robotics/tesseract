/**
 * @page basic_property_tree_example Tesseract PropertyTree Basic Example
 *
 * @section basic_property_tree_overview Overview
 *
 * This example demonstrates the fundamental concepts of Tesseract's PropertyTree system,
 * which provides a hierarchical structure for managing YAML-based configurations with
 * built-in metadata and validation support.
 *
 * PropertyTree is useful for:
 * - Defining schema for configuration files
 * - Merging user configs into schemas with automatic default application
 * - Validating configuration data against schemas
 * - Collecting all validation errors at once (not just the first one)
 * - Attaching metadata (docs, GUI hints, constraints) to configuration properties
 *
 * @section basic_property_tree_concepts Key Concepts
 *
 * - **PropertyTree Node**: A single node representing a property. Can have a value
 *   (YAML::Node), metadata attributes (doc, type, required, etc.), and child nodes.
 *
 * - **Schema**: A PropertyTree that defines the structure and constraints of a config.
 *   Typically created with PropertyTreeBuilder for clean, fluent syntax.
 *
 * - **PropertyTreeBuilder**: A fluent API for constructing PropertyTree schemas with
 *   minimal boilerplate. Supports type methods (string, integer, boolean, etc.) and
 *   attribute setters (doc, required, defaultVal, minimum, maximum, etc.).
 *
 * - **Config Merge**: Populates schema tree with values from a user-supplied YAML config,
 *   applying defaults for missing properties. Tracks extra properties for validation.
 *
 * - **Validation**: Recursively checks all constraints (required, enum, range, type,
 *   custom validators) and collects ALL error messages at once for comprehensive feedback.
 *
 * - **Attributes**: Metadata attached to nodes. Built-in attributes include:
 *   - Type information (TYPE)
 *   - Constraints (REQUIRED, ENUM, MINIMUM, MAXIMUM)
 *   - Defaults (DEFAULT)
 *   - GUI metadata (LABEL, PLACEHOLDER, GROUP, READ_ONLY, HIDDEN, DOC)
 *
 * @section basic_property_tree_workflow Typical Workflow
 *
 * The example demonstrates:
 *
 * 1. **Build a Schema** using PropertyTreeBuilder with types and constraints
 * 2. **Load Configuration** from a YAML node
 * 3. **Merge Config into Schema** to populate values and apply defaults
 * 4. **Validate** to check all constraints, collecting all errors
 * 5. **Access Results** from the merged, validated schema tree
 *
 * @section basic_property_tree_code Example Code
 *
 * @snippet property_tree_basic_example.cpp schema_builder_start
 *
 * The PropertyTreeBuilder fluent API makes it easy to define schemas without
 * nested boilerplate. Type methods create children, attribute setters configure
 * the current node, and done() pops back to the parent.
 *
 * @snippet property_tree_basic_example.cpp merge_validate_start
 *
 * After defining a schema, merge a user config into it. The mergeConfig() call
 * populates values and applies defaults. Then validate() checks all constraints
 * and returns a vector of error strings (empty on success).
 *
 * @section basic_property_tree_run Running the Example
 *
 * Execute the example:
 * @code
 * ./tesseract_common_basic_property_tree_example
 * @endcode
 *
 * The example creates valid and invalid configurations, demonstrating both
 * successful merges and comprehensive error collection.
 */

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <tesseract/common/property_tree.h>

using namespace tesseract::common;
using namespace tesseract::common::property_attribute;
using namespace tesseract::common::property_type;

int main(int /*argc*/, char** /*argv*/)
{
  // ====================================================================
  //! [schema_builder_start]
  // Build a schema for a robot configuration
  // ====================================================================
  // clang-format off
  auto schema = PropertyTreeBuilder()
                    .attribute(TYPE, CONTAINER)
                    .string("name").required().doc("Robot name").label("Robot Name").done()
                    .integer("dof").required().doc("Degrees of freedom").minimum(1).maximum(20).done()
                    .doubleNum("speed").doc("Max speed (units/sec)").defaultVal(1.0)
                      .minimum(0.0).maximum(10.0).done()
                    .container("workspace").doc("Work envelope")
                      .doubleNum("x_min").required().done()
                      .doubleNum("x_max").required().done()
                      .doubleNum("y_min").required().done()
                      .doubleNum("y_max").required().done()
                      .done()
                    .build();
  // clang-format on
  //! [schema_builder_start]

  std::cout << "========================================\n";
  std::cout << "         PropertyTree Basic Example      \n";
  std::cout << "========================================\n\n";

  // ====================================================================
  // Example 1: Valid Configuration
  // ====================================================================
  std::cout << "Example 1: Valid Configuration\n";
  std::cout << "--------------------------------\n";

  //! [merge_validate_start]
  YAML::Node valid_config;
  valid_config["name"] = "UR10";
  valid_config["dof"] = 6;
  valid_config["workspace"]["x_min"] = -2.0;
  valid_config["workspace"]["x_max"] = 2.0;
  valid_config["workspace"]["y_min"] = -3.0;
  valid_config["workspace"]["y_max"] = 3.0;

  // Make a copy of schema for each test
  auto schema_copy = schema;
  schema_copy.mergeConfig(valid_config);
  auto errors = schema_copy.validate();
  //! [merge_validate_start]

  if (errors.empty())
  {
    std::cout << "✓ Validation passed!\n";
    std::cout << "  Robot name: " << schema_copy.at("name").as<std::string>() << "\n";
    std::cout << "  DOF: " << schema_copy.at("dof").as<int>() << "\n";
    std::cout << "  Speed: " << schema_copy.at("speed").as<double>() << " (default applied)\n";
    std::cout << "  Workspace X: [" << schema_copy.at("workspace").at("x_min").as<double>() << ", "
              << schema_copy.at("workspace").at("x_max").as<double>() << "]\n";
  }
  else
  {
    std::cout << "✗ Validation failed with " << errors.size() << " errors:\n";
    for (const auto& err : errors)
      std::cout << "  - " << err << "\n";
  }

  // ====================================================================
  // Example 2: Missing Required Field
  // ====================================================================
  std::cout << "\nExample 2: Missing Required Field\n";
  std::cout << "----------------------------------\n";

  YAML::Node missing_required;
  missing_required["name"] = "UR5";
  // missing 'dof' (required)
  missing_required["workspace"]["x_min"] = 0.0;
  missing_required["workspace"]["x_max"] = 1.0;
  missing_required["workspace"]["y_min"] = 0.0;
  missing_required["workspace"]["y_max"] = 1.0;

  schema_copy = schema;
  schema_copy.mergeConfig(missing_required);
  errors = schema_copy.validate();

  if (!errors.empty())
  {
    std::cout << "✗ Validation found " << errors.size() << " error(s):\n";
    for (const auto& err : errors)
      std::cout << "  - " << err << "\n";
  }

  // ====================================================================
  // Example 3: Out-of-Range Value
  // ====================================================================
  std::cout << "\nExample 3: Out-of-Range Value\n";
  std::cout << "------------------------------\n";

  YAML::Node out_of_range;
  out_of_range["name"] = "UR20";
  out_of_range["dof"] = 25;  // exceeds maximum of 20
  out_of_range["workspace"]["x_min"] = 0.0;
  out_of_range["workspace"]["x_max"] = 1.0;
  out_of_range["workspace"]["y_min"] = 0.0;
  out_of_range["workspace"]["y_max"] = 1.0;

  schema_copy = schema;
  schema_copy.mergeConfig(out_of_range);
  errors = schema_copy.validate();

  if (!errors.empty())
  {
    std::cout << "✗ Validation found " << errors.size() << " error(s):\n";
    for (const auto& err : errors)
      std::cout << "  - " << err << "\n";
  }

  // ====================================================================
  // Example 4: Multiple Errors
  // ====================================================================
  std::cout << "\nExample 4: Multiple Errors (Comprehensive Feedback)\n";
  std::cout << "----------------------------------------------------\n";

  YAML::Node multiple_errors;
  multiple_errors["dof"] = 100;     // exceeds maximum
  multiple_errors["speed"] = -5.0;  // exceeds minimum
  // missing 'name' (required)
  // missing 'workspace' (required subfields)

  schema_copy = schema;
  schema_copy.mergeConfig(multiple_errors);
  errors = schema_copy.validate();

  std::cout << "✗ Validation found " << errors.size() << " error(s):\n";
  for (const auto& err : errors)
    std::cout << "  - " << err << "\n";

  std::cout << "\nNote: PropertyTree collects ALL errors at once,\n";
  std::cout << "giving comprehensive feedback instead of stopping\n";
  std::cout << "at the first error.\n";

  // ====================================================================
  // Example 5: Schema Inspection
  // ====================================================================
  std::cout << "\nExample 5: Schema Inspection (Metadata Access)\n";
  std::cout << "----------------------------------------------\n";

  const auto& dof_node = schema.at("dof");
  auto type_attr = dof_node.getAttribute(TYPE);
  auto min_attr = dof_node.getAttribute(MINIMUM);
  auto max_attr = dof_node.getAttribute(MAXIMUM);
  auto doc_attr = dof_node.getAttribute(DOC);

  std::cout << "Property: dof\n";
  if (type_attr)
    std::cout << "  Type: " << type_attr->as<std::string>() << "\n";
  if (doc_attr)
    std::cout << "  Doc: " << doc_attr->as<std::string>() << "\n";
  if (min_attr)
    std::cout << "  Minimum: " << min_attr->as<int>() << "\n";
  if (max_attr)
    std::cout << "  Maximum: " << max_attr->as<int>() << "\n";

  std::cout << "\n========================================\n";
  std::cout << "         Example Complete\n";
  std::cout << "========================================\n";

  return 0;
}
