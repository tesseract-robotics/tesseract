/**
 * @page oneof_property_tree_example Tesseract PropertyTree OneOf Example
 *
 * @section oneof_property_tree_overview Overview
 *
 * This example demonstrates the oneOf feature of Tesseract's PropertyTree system,
 * which enables polymorphic configuration structures. A oneOf schema allows exactly
 * one of multiple configuration branches to be active at a time.
 *
 * OneOf is useful for:
 * - Union-like types (choose one of several variants)
 * - Mutually exclusive configurations
 * - Plugin selection (use either plugin A or plugin B)
 * - Shape definitions (circle OR rectangle)
 * - Protocol selection (USB OR Ethernet with type-specific fields)
 *
 * @section oneof_property_tree_concepts Key Concepts
 *
 * - **Branches**: Named child schemas, each defining a configuration variant
 * - **Branch Selection**: Automatic selection based on which required fields are present
 * - **Flattening**: After selection, the schema becomes the selected branch
 * - **One Mandatory Match**: Exactly one branch must match, no more, no less
 * - **Validation**: Constraints validate only in the selected branch
 *
 * @section oneof_property_tree_workflow OneOf Workflow
 *
 * The example demonstrates:
 *
 * 1. **Define a OneOf Schema** with multiple branches
 * 2. **Load Configuration** specifying one branch
 * 3. **Merge Config into Schema** - branch selection occurs here
 * 4. **Validate** - checks constraints for selected branch only
 * 5. **Extract Results** from the flattened schema
 *
 * @section oneof_property_tree_code Example Code
 *
 * @snippet property_tree_oneof_example.cpp oneof_schema_start
 *
 * The @link tesseract::common::property_attribute::ONEOF ONEOF @endlink type defines
 * a schema where exactly one child must be selected. Child nodes are branches.
 *
 * @snippet property_tree_oneof_example.cpp oneof_merge_start
 *
 * During mergeConfig(), PropertyTree examines the config and selects the branch
 * whose required fields match the provided keys. If multiple branches match or none
 * match, an exception is thrown.
 *
 * @section oneof_property_tree_run Running the Example
 *
 * Execute the example:
 * @code
 * ./tesseract_common_oneof_property_tree_example
 * @endcode
 *
 * The example demonstrates successful branch selection, error cases, and
 * how validation applies only to the selected branch.
 */

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <tesseract/common/property_tree.h>

using namespace tesseract::common;
using namespace tesseract::common::property_attribute;
using namespace tesseract::common::property_type;

int main(int /*argc*/, char** /*argv*/)
{
  std::cout << "========================================\n";
  std::cout << "      PropertyTree OneOf Example        \n";
  std::cout << "========================================\n\n";

  // ====================================================================
  // Example 1: Shape Definition - Circle vs Rectangle
  // ====================================================================
  std::cout << "Example 1: Shape Definition (Circle vs Rectangle)\n";
  std::cout << "------------------------------------------------\n";

  //! [oneof_schema_start]
  // clang-format off
  auto shape_schema = PropertyTreeBuilder()
      .attribute(TYPE, ONEOF)
      .container("circle")
        .float64("radius").required()
          .doc("Circle radius").minimum(0.0).label("Radius").done()
        .done()
      .container("rectangle")
        .float64("width").required()
          .doc("Rectangle width").minimum(0.0).label("Width").done()
        .float64("height").required()
          .doc("Rectangle height").minimum(0.0).label("Height").done()
        .done()
      .build();
  // clang-format on
  //! [oneof_schema_start]

  //! [oneof_merge_start]
  // Example 1a: Circle configuration
  {
    std::cout << "\n  Circle config:\n";

    YAML::Node circle_config;
    circle_config["radius"] = 5.5;

    auto schema_copy = shape_schema;
    try
    {
      schema_copy.mergeConfig(circle_config);
      auto errors = schema_copy.validate();

      if (errors.empty())
      {
        std::cout << "    ✓ Selected circle branch\n";
        std::cout << "    Radius: " << schema_copy.at("radius").as<double>() << "\n";
      }
      else
      {
        std::cout << "    ✗ Validation errors:\n";
        for (const auto& e : errors)
          std::cout << "      - " << e << "\n";
      }
    }
    catch (const std::exception& ex)
    {
      std::cout << "    ✗ Merge failed: " << ex.what() << "\n";
    }
  }
  //! [oneof_merge_start]

  // Example 1b: Rectangle configuration
  {
    std::cout << "\n  Rectangle config:\n";

    YAML::Node rect_config;
    rect_config["width"] = 10.0;
    rect_config["height"] = 20.0;

    auto schema_copy = shape_schema;
    try
    {
      schema_copy.mergeConfig(rect_config);
      auto errors = schema_copy.validate();

      if (errors.empty())
      {
        std::cout << "    ✓ Selected rectangle branch\n";
        std::cout << "    Width: " << schema_copy.at("width").as<double>() << "\n";
        std::cout << "    Height: " << schema_copy.at("height").as<double>() << "\n";
      }
    }
    catch (const std::exception& ex)
    {
      std::cout << "    ✗ Merge failed: " << ex.what() << "\n";
    }
  }

  // ====================================================================
  // Example 2: Plugin Selection - USB or Ethernet
  // ====================================================================
  std::cout << "\nExample 2: Plugin Selection (USB vs Ethernet)\n";
  std::cout << "-------------------------------------------\n";

  // clang-format off
  auto plugin_schema = PropertyTreeBuilder()
      .attribute(TYPE, ONEOF)
      .container("usb_plugin")
        .string("port").required()
          .doc("Serial port").placeholder("/dev/ttyUSB0").label("Port").done()
        .int32("baud_rate").required()
          .doc("Serial baud rate").minimum(300).maximum(921600).label("Baud Rate").done()
        .done()
      .container("ethernet_plugin")
        .string("ip_address").required()
          .doc("Remote IP address").placeholder("192.168.1.1").label("IP Address").done()
        .int32("port").required()
          .doc("TCP port").minimum(1024).maximum(65535).label("Port").done()
        .boolean("use_ipv6")
          .doc("Use IPv6 if available").defaultVal(false).label("IPv6").done()
        .done()
      .build();
  // clang-format on

  // Example 2a: USB plugin
  {
    std::cout << "\n  USB plugin config:\n";

    YAML::Node usb_config;
    usb_config["port"] = "/dev/ttyUSB0";
    usb_config["baud_rate"] = 115200;

    auto schema_copy = plugin_schema;
    try
    {
      schema_copy.mergeConfig(usb_config);
      auto errors = schema_copy.validate();

      if (errors.empty())
      {
        std::cout << "    ✓ Selected USB plugin\n";
        std::cout << "    Port: " << schema_copy.at("port").as<std::string>() << "\n";
        std::cout << "    Baud: " << schema_copy.at("baud_rate").as<int32_t>() << "\n";
      }
    }
    catch (const std::exception& ex)
    {
      std::cout << "    ✗ Merge failed: " << ex.what() << "\n";
    }
  }

  // Example 2b: Ethernet plugin with defaults
  {
    std::cout << "\n  Ethernet plugin config:\n";

    YAML::Node eth_config;
    eth_config["ip_address"] = "192.168.1.100";
    eth_config["port"] = 5005;

    auto schema_copy = plugin_schema;
    try
    {
      schema_copy.mergeConfig(eth_config);
      auto errors = schema_copy.validate();

      if (errors.empty())
      {
        std::cout << "    ✓ Selected Ethernet plugin\n";
        std::cout << "    IP: " << schema_copy.at("ip_address").as<std::string>() << "\n";
        std::cout << "    Port: " << schema_copy.at("port").as<int32_t>() << "\n";
        std::cout << "    IPv6: " << (schema_copy.at("use_ipv6").as<bool>() ? "enabled" : "disabled") << " (default)\n";
      }
    }
    catch (const std::exception& ex)
    {
      std::cout << "    ✗ Merge failed: " << ex.what() << "\n";
    }
  }

  // ====================================================================
  // Example 3: Ambiguous Config - No Branch Matches
  // ====================================================================
  std::cout << "\nExample 3: No Matching Branch\n";
  std::cout << "-----------------------------\n";

  {
    YAML::Node bad_config;
    bad_config["unknown_field"] = "value";

    auto schema_copy = shape_schema;
    try
    {
      schema_copy.mergeConfig(bad_config);
      std::cout << "  This shouldn't print\n";
    }
    catch (const std::exception& ex)
    {
      std::cout << "  ✓ Correctly caught error:\n";
      std::cout << "    " << ex.what() << "\n";
    }
  }

  // ====================================================================
  // Example 4: Branch Validation After Selection
  // ====================================================================
  std::cout << "\nExample 4: Validation in Selected Branch\n";
  std::cout << "---------------------------------------\n";

  {
    std::cout << "\n  Rectangle with invalid dimensions:\n";

    YAML::Node bad_rect;
    bad_rect["width"] = -5.0;  // violates minimum(0.0)
    bad_rect["height"] = 0.0;  // violates minimum(0.0)

    auto schema_copy = shape_schema;
    try
    {
      schema_copy.mergeConfig(bad_rect);
      auto errors = schema_copy.validate();

      std::cout << "    ✓ Selected rectangle branch\n";
      if (!errors.empty())
      {
        std::cout << "    ✗ Validation found " << errors.size() << " error(s):\n";
        for (const auto& e : errors)
          std::cout << "      - " << e << "\n";
      }
    }
    catch (const std::exception& ex)
    {
      std::cout << "    ✗ Merge failed: " << ex.what() << "\n";
    }
  }

  // ====================================================================
  // Example 5: Sensor Type Selection
  // ====================================================================
  std::cout << "\nExample 5: Sensor Configuration\n";
  std::cout << "------------------------------\n";

  // clang-format off
  auto sensor_schema = PropertyTreeBuilder()
      .attribute(TYPE, ONEOF)
      .container("camera")
        .string("device").required().label("Device Path").placeholder("/dev/video0").done()
        .int32("fps").required().minimum(1).maximum(120).label("FPS").done()
        .string("resolution").defaultVal("1280x720").label("Resolution").done()
        .done()
      .container("lidar")
        .string("device").required().label("Device Path").placeholder("/dev/ttyACM0").done()
        .int32("scan_rate").required().minimum(1).maximum(100).label("Scan Rate (Hz)").done()
        .float64("range").required().minimum(0.1).maximum(200.0).label("Max Range (m)").done()
        .done()
      .container("imu")
        .string("device").required().label("Device Path").placeholder("/dev/ttyUSB0").done()
        .int32("sample_rate").required().minimum(10).maximum(10000).label("Sample Rate (Hz)").done()
        .done()
      .build();
  // clang-format on

  {
    std::cout << "\n  Camera sensor:\n";

    YAML::Node camera_config;
    camera_config["device"] = "/dev/video0";
    camera_config["fps"] = 30;

    auto schema_copy = sensor_schema;
    try
    {
      schema_copy.mergeConfig(camera_config);
      auto errors = schema_copy.validate();

      if (errors.empty())
      {
        std::cout << "    ✓ Selected camera branch\n";
        std::cout << "    Device: " << schema_copy.at("device").as<std::string>() << "\n";
        std::cout << "    FPS: " << schema_copy.at("fps").as<int32_t>() << "\n";
        std::cout << "    Resolution: " << schema_copy.at("resolution").as<std::string>() << " (default)\n";
      }
    }
    catch (const std::exception& ex)
    {
      std::cout << "    ✗ Failed: " << ex.what() << "\n";
    }
  }

  {
    std::cout << "\n  LiDAR sensor:\n";

    YAML::Node lidar_config;
    lidar_config["device"] = "/dev/ttyACM0";
    lidar_config["scan_rate"] = 10;
    lidar_config["range"] = 100.0;

    auto schema_copy = sensor_schema;
    try
    {
      schema_copy.mergeConfig(lidar_config);
      auto errors = schema_copy.validate();

      if (errors.empty())
      {
        std::cout << "    ✓ Selected lidar branch\n";
        std::cout << "    Device: " << schema_copy.at("device").as<std::string>() << "\n";
        std::cout << "    Scan Rate: " << schema_copy.at("scan_rate").as<int32_t>() << " Hz\n";
        std::cout << "    Max Range: " << schema_copy.at("range").as<double>() << " m\n";
      }
    }
    catch (const std::exception& ex)
    {
      std::cout << "    ✗ Failed: " << ex.what() << "\n";
    }
  }

  std::cout << "\n========================================\n";
  std::cout << "         Example Complete\n";
  std::cout << "========================================\n";

  return 0;
}
