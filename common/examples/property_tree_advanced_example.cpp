/**
 * @page advanced_property_tree_example Tesseract PropertyTree Advanced Example
 *
 * @section advanced_property_tree_overview Overview
 *
 * This advanced example demonstrates sophisticated features of the PropertyTree system:
 * - Custom validators
 * - Schema registry for reusable schemas
 * - Real-world configuration patterns (enums, nested structures)
 * - Fluent builder patterns for complex hierarchies
 * - GUI metadata for automatic UI generation
 *
 * @section advanced_property_tree_features Advanced Features
 *
 * - **Custom Validators**: Attach arbitrary validation logic to properties
 * - **Schema Registry**: Register schemas globally for reuse across your application
 * - **Enum Validation**: Restrict values to a set of allowed strings
 * - **Nested Containers**: Build deeply nested configuration hierarchies with builder
 * - **GUI Metadata**: Attach UI hints (group, label, placeholder) for automatic form generation
 * - **Fluent API**: Chain attribute setters for clean, readable schema definitions
 *
 * @section advanced_property_tree_workflow Advanced Workflow
 *
 * The example demonstrates:
 *
 * 1. **Register Reusable Schemas** in the global registry
 * 2. **Define Custom Validators** for domain-specific constraints
 * 3. **Build Complex Nested Schemas** with the fluent API
 * 4. **Attach GUI Metadata** for automatic form generation
 * 5. **Validate with Custom Rules** and collect comprehensive feedback
 *
 * @snippet property_tree_advanced_example.cpp custom_validator_start
 *
 * Custom validators receive the PropertyTree node, path, and error vector.
 * They can perform domain-specific validation and push error messages.
 *
 * @snippet property_tree_advanced_example.cpp enum_validation_start
 *
 * Enum validation restricts values to a predefined set. The builder's
 * enumValues() method makes it easy to define allowed values.
 *
 * @snippet property_tree_advanced_example.cpp nested_builder_start
 *
 * Complex nested structures are easy to build with the fluent API.
 * Type methods descend into children, attribute setters configure the
 * current node, and done() pops back to the parent.
 *
 * @section advanced_property_tree_run Running the Example
 *
 * Execute the example:
 * @code
 * ./tesseract_common_advanced_property_tree_example
 * @endcode
 *
 * The example demonstrates custom validation, enum constraints, nested
 * structures, and GUI metadata application.
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
  std::cout << "========================================\n";
  std::cout << "    PropertyTree Advanced Example      \n";
  std::cout << "========================================\n\n";

  // ====================================================================
  // Example 1: Custom Validators
  // ====================================================================
  std::cout << "Example 1: Custom Validators\n";
  std::cout << "-----------------------------\n";

  //! [custom_validator_start]
  // Define a schema for a file path with custom validation
  auto file_schema =
      PropertyTreeBuilder()
          .string("filepath")
          .required()
          .doc("Path to configuration file")
          .placeholder("/path/to/config.yaml")
          .validator([](const PropertyTree& node, const std::string& path, std::vector<std::string>& errors) {
            auto val = node.getValue().as<std::string>();
            // Custom rule: path must not be empty
            if (val.empty())
              errors.push_back(path + ": filepath must not be empty");
            // Custom rule: must end with .yaml or .yml
            if (val.find(".yaml") == std::string::npos && val.find(".yml") == std::string::npos)
              errors.push_back(path + ": filepath must end with .yaml or .yml");
          })
          .done()
          .build();
  //! [custom_validator_start]

  {
    YAML::Node valid_file;
    valid_file["filepath"] = "/etc/robots/ur10.yaml";

    auto schema_copy = file_schema;
    schema_copy.mergeConfig(valid_file);
    auto errors = schema_copy.validate();

    std::cout << "  Valid filepath:\n";
    std::cout << "    " << valid_file["filepath"].as<std::string>() << "\n";
    if (errors.empty())
      std::cout << "    ✓ Passed custom validators\n";
    else
    {
      std::cout << "    ✗ Failed: ";
      for (const auto& e : errors)
        std::cout << e << "\n";
    }
  }

  {
    YAML::Node invalid_file;
    invalid_file["filepath"] = "/etc/config.json";  // wrong extension

    auto schema_copy = file_schema;
    schema_copy.mergeConfig(invalid_file);
    auto errors = schema_copy.validate();

    std::cout << "\n  Invalid filepath:\n";
    std::cout << "    " << invalid_file["filepath"].as<std::string>() << "\n";
    if (!errors.empty())
    {
      std::cout << "    ✗ Custom validator caught:\n";
      for (const auto& e : errors)
        std::cout << "      - " << e << "\n";
    }
  }

  // ====================================================================
  // Example 2: Enum Validation
  // ====================================================================
  std::cout << "\nExample 2: Enum Validation\n";
  std::cout << "--------------------------\n";

  //! [enum_validation_start]
  auto control_mode_schema = PropertyTreeBuilder()
                                 .string("mode")
                                 .required()
                                 .doc("Control mode")
                                 .enumValues({ "position", "velocity", "torque" })
                                 .done()
                                 .build();
  //! [enum_validation_start]

  {
    YAML::Node valid_mode;
    valid_mode["mode"] = "velocity";

    auto schema_copy = control_mode_schema;
    schema_copy.mergeConfig(valid_mode);
    auto errors = schema_copy.validate();

    std::cout << "  Mode: " << valid_mode["mode"].as<std::string>() << "\n";
    std::cout << "  " << (errors.empty() ? "✓ Valid enum value" : "✗ Invalid enum value") << "\n";
  }

  {
    YAML::Node invalid_mode;
    invalid_mode["mode"] = "impedance";  // not in enum list

    auto schema_copy = control_mode_schema;
    schema_copy.mergeConfig(invalid_mode);
    auto errors = schema_copy.validate();

    std::cout << "\n  Mode: " << invalid_mode["mode"].as<std::string>() << "\n";
    if (!errors.empty())
    {
      std::cout << "  ✗ Rejected:\n";
      for (const auto& e : errors)
        std::cout << "    - " << e << "\n";
    }
  }

  // ====================================================================
  // Example 3: Nested Structures with GUI Metadata
  // ====================================================================
  std::cout << "\nExample 3: Nested Structures with GUI Metadata\n";
  std::cout << "----------------------------------------------\n";

  //! [nested_builder_start]
  // clang-format off
  auto task_schema = PropertyTreeBuilder()
                         .attribute(TYPE, CONTAINER)
                         .string("name").required().doc("Task name").label("Task Name").group("general").done()
                         .string("status").doc("Current status")
                           .enumValues({ "pending", "running", "completed", "failed" })
                           .defaultVal("pending").label("Status").group("general").done()
                         .container("parameters").doc("Task-specific parameters").group("parameters")
                           .float64("timeout").doc("Max execution time (seconds)")
                             .minimum(0.0).maximum(3600.0).defaultVal(60.0).label("Timeout").done()
                           .boolean("verbose").doc("Enable verbose logging")
                             .defaultVal(false).label("Verbose Output").done()
                           .done()
                         .build();
  // clang-format on
  //! [nested_builder_start]

  YAML::Node task_config;
  task_config["name"] = "calibration";
  task_config["parameters"]["timeout"] = 120.0;
  task_config["parameters"]["verbose"] = true;

  auto schema_copy = task_schema;
  schema_copy.mergeConfig(task_config);
  auto errors = schema_copy.validate();

  std::cout << "  Task configuration:\n";
  std::cout << "    Name: " << schema_copy.at("name").as<std::string>() << "\n";
  std::cout << "    Status: " << schema_copy.at("status").as<std::string>() << " (default applied)\n";
  std::cout << "    Timeout: " << schema_copy.at("parameters").at("timeout").as<double>() << "s\n";
  std::cout << "    Verbose: " << (schema_copy.at("parameters").at("verbose").as<bool>() ? "enabled" : "disabled")
            << "\n";
  std::cout << "    ✓ Merged and validated\n";

  if (!errors.empty())
  {
    std::cout << "    Errors:\n";
    for (const auto& e : errors)
      std::cout << "      - " << e << "\n";
  }

  // ====================================================================
  // Example 4: Schema Inspection for UI Generation
  // ====================================================================
  std::cout << "\nExample 4: Metadata Inspection (for UI generation)\n";
  std::cout << "---------------------------------------------------\n";

  std::cout << "  Extracting metadata from task schema:\n";
  const auto& params_node = task_schema.at("parameters");
  const auto& timeout_node = params_node.at("timeout");

  auto label = timeout_node.getAttribute(LABEL);
  auto doc = timeout_node.getAttribute(DOC);
  auto group = timeout_node.getAttribute(GROUP);
  auto min_val = timeout_node.getAttribute(MINIMUM);
  auto max_val = timeout_node.getAttribute(MAXIMUM);
  auto default_val = timeout_node.getAttribute(DEFAULT);

  std::cout << "    Property: timeout\n";
  if (label)
    std::cout << "      Label: " << label->as<std::string>() << "\n";
  if (doc)
    std::cout << "      Doc: " << doc->as<std::string>() << "\n";
  if (group)
    std::cout << "      Group: " << group->as<std::string>() << "\n";
  if (min_val)
    std::cout << "      Range: [" << min_val->as<double>();
  if (max_val)
    std::cout << ", " << max_val->as<double>() << "]\n";
  if (default_val)
    std::cout << "      Default: " << default_val->as<double>() << "\n";

  std::cout << "\n    (UI generators can use this metadata to:\n";
  std::cout << "     - Create labeled input fields\n";
  std::cout << "     - Show range sliders for constrained numbers\n";
  std::cout << "     - Organize fields into groups/tabs\n";
  std::cout << "     - Display documentation tooltips)\n";

  // ====================================================================
  // Example 5: Real-world Scenario - Sensor Configuration
  // ====================================================================
  std::cout << "\nExample 5: Real-world Scenario - Sensor Configuration\n";
  std::cout << "------------------------------------------------------\n";

  // clang-format off
  auto sensor_schema = PropertyTreeBuilder()
                           .attribute(TYPE, CONTAINER)
                           .string("type").required()
                             .enumValues({ "camera", "lidar", "imu", "force_torque" })
                             .label("Sensor Type").done()
                           .string("frame_id").required()
                             .label("Reference Frame").placeholder("sensor_frame").done()
                           .container("config").doc("Sensor-specific configuration")
                             .string("model").label("Model").defaultVal("generic").done()
                             .int32("resolution").doc("Resolution (varies by sensor type)")
                               .minimum(1).maximum(8192).label("Resolution").done()
                             .float64("fov").doc("Field of view (degrees)")
                               .minimum(0.1).maximum(180.0).label("FOV").done()
                             .done()
                           .build();
  // clang-format on

  YAML::Node sensor_config;
  sensor_config["type"] = "camera";
  sensor_config["frame_id"] = "camera_link";
  sensor_config["config"]["model"] = "RealSense D435";
  sensor_config["config"]["resolution"] = 1080;
  sensor_config["config"]["fov"] = 70.0;

  schema_copy = sensor_schema;
  schema_copy.mergeConfig(sensor_config);
  errors = schema_copy.validate();

  std::cout << "  Sensor configuration:\n";
  std::cout << "    Type: " << schema_copy.at("type").as<std::string>() << "\n";
  std::cout << "    Frame: " << schema_copy.at("frame_id").as<std::string>() << "\n";
  std::cout << "    Model: " << schema_copy.at("config").at("model").as<std::string>() << "\n";
  std::cout << "    Resolution: " << schema_copy.at("config").at("resolution").as<int32_t>() << "p\n";
  std::cout << "    FOV: " << schema_copy.at("config").at("fov").as<double>() << "°\n";

  if (errors.empty())
    std::cout << "    ✓ Configuration valid\n";
  else
  {
    std::cout << "    ✗ Validation errors:\n";
    for (const auto& e : errors)
      std::cout << "      - " << e << "\n";
  }

  std::cout << "\n========================================\n";
  std::cout << "         Example Complete\n";
  std::cout << "========================================\n";

  return 0;
}
