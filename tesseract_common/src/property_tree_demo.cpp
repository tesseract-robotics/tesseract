#include <iostream>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/property_tree.h>
#include <tesseract_common/utils.h>

using namespace tesseract_common;

/// Build a detailed schema for 'config', attaching free-function validators.
PropertyTree buildConfigSchema()
{
  PropertyTree schema;
  auto& cfg = schema.get("config");
  cfg.setAttribute(property_attribute::DOC, "Main config for plugin");
  cfg.setAttribute(property_attribute::REQUIRED, true);
  std::map<int, std::string> return_options;
  return_options[0] = "Error";
  return_options[1] = "Successful";
  cfg.setAttribute("return_options", YAML::Node(return_options));
  cfg.addValidator(validateRequired);

  // conditional
  {
    auto& prop = cfg.get("conditional");
    prop.setAttribute(property_attribute::TYPE, property_type::BOOL);
    prop.setAttribute(property_attribute::DEFAULT, true);
    prop.setAttribute(property_attribute::DOC, "Enable conditional execution");
    prop.setAttribute(property_attribute::REQUIRED, true);
    prop.addValidator(validateRequired);
  }
  // inputs
  {
    auto& inputs = cfg.get("inputs");
    inputs.setAttribute(property_attribute::DOC, "Input sources");
    inputs.setAttribute(property_attribute::REQUIRED, true);
    // program
    {
      auto& prop = inputs.get("program");
      prop.setAttribute(property_attribute::TYPE, property_type::STRING);
      prop.setAttribute(property_attribute::DOC, "The composite instruction");
      prop.setAttribute(property_attribute::REQUIRED, true);
      prop.addValidator(validateRequired);
    }
    // environment
    {
      auto& prop = inputs.get("environment");
      // env.setAttribute("enum", YAML::Load(R"(["dev","stag","prod"])"));
      prop.setAttribute(property_attribute::TYPE, property_type::STRING);
      prop.setAttribute(property_attribute::DOC, "The tesseract environment");
      prop.setAttribute(property_attribute::REQUIRED, true);
      prop.addValidator(validateRequired);
    }
    // profiles
    {
      auto& prop = inputs.get("profiles");
      prop.setAttribute(property_attribute::TYPE, property_type::STRING);
      prop.setAttribute(property_attribute::DOC, "The tesseract profiles");
      prop.setAttribute(property_attribute::REQUIRED, true);
      prop.addValidator(validateRequired);
      // prof.setAttribute("enum", YAML::Load(R"(["A","B"])"));
      //   prof.addValidator(validateEnum);
      //   prof.addValidator([](auto const& node, auto& errs, const auto& path){
      //     auto s = node.getValue().template as<std::string>();
      //     if (s.find(',') == std::string::npos) {
      //       errs.push_back(path + ": profiles should be comma-separated");
      //       return false;
      //     }
      //     return true;
      //   });
    }
  }
  // outputs
  {
    auto& outs = cfg.get("outputs");
    auto& prop = outs.get("program");
    prop.setAttribute(property_attribute::TYPE, property_type::STRING);
    prop.setAttribute(property_attribute::REQUIRED, true);
    prop.addValidator(validateRequired);
  }
  // format_result_as_input
  {
    auto& prop = cfg.get("format_result_as_input");
    prop.setAttribute(property_attribute::TYPE, property_type::BOOL);
    prop.setAttribute(property_attribute::DEFAULT, false);
  }
  return schema;
}

std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             program: output_data
                           format_result_as_input: false)";

std::string str2 = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             program: output_data)";

std::string str3 = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             programs: output_data)";

int main()
{
  // Load configuration from YAML
  PropertyTree prop = PropertyTree::fromYAML(YAML::Load(str2));

  // Parse schema from external file (config_schema.yaml)
  PropertyTree schema = buildConfigSchema();

  // Merge schema metadata into config tree
  prop.mergeSchema(schema);
  std::cout << "Exclude attrubutes:\n" << prop.toYAML() << "\n\n";
  std::cout << "Include attrubutes:\n" << prop.toYAML(false) << "\n\n";

  try
  {
    prop.validate();
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
    return 1;
  }

  bool cond = prop.get("config").get("conditional").getValue().as<bool>();
  std::cout << "conditional = " << std::boolalpha << cond << "\n";
  return 0;
}
