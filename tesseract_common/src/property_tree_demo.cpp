#include <iostream>
#include <yaml-cpp/yaml.h>
#include <tesseract_common/property_tree.h>
#include <tesseract_common/schema_registry.h>
#include <tesseract_common/schema_registration.h>
#include <tesseract_common/utils.h>

using namespace tesseract_common;

/// Build a detailed schema for 'config', attaching free-function validators.
PropertyTree buildConfigSchema()
{
  PropertyTree schema;
  auto& cfg = schema.get("config");
  cfg.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
  cfg.setAttribute(property_attribute::DOC, "Main config for plugin");
  cfg.setAttribute(property_attribute::REQUIRED, true);
  std::map<int, std::string> return_options;
  return_options[0] = "Error";
  return_options[1] = "Successful";
  cfg.setAttribute("return_options", YAML::Node(return_options));

  // conditional
  {
    auto& prop = cfg.get("conditional");
    prop.setAttribute(property_attribute::TYPE, property_type::BOOL);
    prop.setAttribute(property_attribute::DEFAULT, true);
    prop.setAttribute(property_attribute::DOC, "Enable conditional execution");
    prop.setAttribute(property_attribute::REQUIRED, true);
  }
  // inputs
  {
    auto& inputs = cfg.get("inputs");
    inputs.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
    inputs.setAttribute(property_attribute::DOC, "Input sources");
    inputs.setAttribute(property_attribute::REQUIRED, true);
    // program
    {
      auto& prop = inputs.get("program");
      prop.setAttribute(property_attribute::TYPE, property_type::STRING);
      prop.setAttribute(property_attribute::DOC, "The composite instruction");
      prop.setAttribute(property_attribute::REQUIRED, true);
    }
    // environment
    {
      auto& prop = inputs.get("environment");
      // env.setAttribute("enum", YAML::Load(R"(["dev","stag","prod"])"));
      prop.setAttribute(property_attribute::TYPE, property_type::STRING);
      prop.setAttribute(property_attribute::DOC, "The tesseract environment");
      prop.setAttribute(property_attribute::REQUIRED, true);
    }
    // profiles
    {
      auto& prop = inputs.get("profiles");
      prop.setAttribute(property_attribute::TYPE, property_type::STRING);
      prop.setAttribute(property_attribute::DOC, "The tesseract profiles");
      prop.setAttribute(property_attribute::REQUIRED, true);
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
    auto& outputs = cfg.get("outputs");
    outputs.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
    outputs.setAttribute(property_attribute::DOC, "Output sources");
    outputs.setAttribute(property_attribute::REQUIRED, true);
    {
      auto& prop = outputs.get("program");
      prop.setAttribute(property_attribute::TYPE, property_type::STRING);
      prop.setAttribute(property_attribute::REQUIRED, true);
    }
  }
  // format_result_as_input
  {
    auto& prop = cfg.get("format_result_as_input");
    prop.setAttribute(property_attribute::TYPE, property_type::BOOL);
    prop.setAttribute(property_attribute::DEFAULT, false);
  }
  return schema;
}

tesseract_common::PropertyTree getTaskComposerGraphSchema()
{
  using namespace tesseract_common;
  PropertyTree schema;
  schema.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
  schema.setAttribute(property_attribute::DOC, "TaskComposerGraph");
  {
    auto& prop = schema.get("class");
    prop.setAttribute(property_attribute::TYPE, property_type::STRING);
    prop.setAttribute(property_attribute::DOC, "The class factory name");
    prop.setAttribute(property_attribute::REQUIRED, true);
  }

  auto& config_schema = schema.get("config");
  config_schema.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
  config_schema.setAttribute(property_attribute::REQUIRED, true);
  {
    auto& prop = config_schema.get("conditional");
    prop.setAttribute(property_attribute::TYPE, property_type::BOOL);
    prop.setAttribute(property_attribute::DEFAULT, true);
    prop.setAttribute(property_attribute::DOC, "Enable conditional execution");
    prop.setAttribute(property_attribute::REQUIRED, true);
  }

  {
    auto& inputs = config_schema.get("inputs");
    inputs.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
    inputs.setAttribute(property_attribute::DOC, "Input sources");
  }

  {
    auto& outputs = config_schema.get("outputs");
    outputs.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
    outputs.setAttribute(property_attribute::DOC, "Output sources");
  }

  {
    auto& prop = config_schema.get("nodes");
    prop.setAttribute(property_attribute::TYPE, "TaskComposerGraphNode{}");
    prop.setAttribute(property_attribute::DOC, "Map of all task nodes");
    prop.setAttribute(property_attribute::REQUIRED, true);
  }

  {
    auto& prop = config_schema.get("edges");
    prop.setAttribute(property_attribute::TYPE, "TaskComposerGraphEdge[]");
    prop.setAttribute(property_attribute::DOC, "List of graph edges");
    prop.setAttribute(property_attribute::REQUIRED, true);
    prop.addValidator(validateCustomType);
  }

  {
    auto& prop = config_schema.get("terminals");
    prop.setAttribute(property_attribute::TYPE, property_type::createList(property_type::STRING));
    prop.setAttribute(property_attribute::DOC, "List of terminal tasks");
    prop.setAttribute(property_attribute::REQUIRED, true);
    prop.addValidator(validateTypeCast<std::vector<std::string>>);
  }

  return schema;
}

tesseract_common::PropertyTree getTaskComposerGraphEdgeSchema()
{
  using namespace tesseract_common;
  PropertyTree schema;
  schema.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
  schema.setAttribute(property_attribute::DOC, "TaskComposerGraphEdge");
  {
    auto& prop = schema.get("source");
    prop.setAttribute(property_attribute::TYPE, property_type::STRING);
    prop.setAttribute(property_attribute::DOC, "The source task name");
    prop.setAttribute(property_attribute::REQUIRED, true);
  }

  {
    auto& prop = schema.get("destinations");
    prop.setAttribute(property_attribute::TYPE, property_type::createList(property_type::STRING));
    prop.setAttribute(property_attribute::DOC, "The list of destination task name");
    prop.setAttribute(property_attribute::REQUIRED, true);
    prop.addValidator(validateTypeCast<std::vector<std::string>>);
  }

  return schema;
}

tesseract_common::PropertyTree getTaskComposerRasterOnlySchema()
{
  using namespace tesseract_common;
  PropertyTree schema;
  schema.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
  schema.setAttribute(property_attribute::DOC, "TaskComposerGraph");
  std::map<int, std::string> return_options;
  return_options[0] = "Error";
  return_options[1] = "Successful";
  schema.setAttribute("return_options", YAML::Node(return_options));
  {
    auto& prop = schema.get("class");
    prop.setAttribute(property_attribute::TYPE, property_type::STRING);
    prop.setAttribute(property_attribute::DOC, "The class factory name");
    prop.setAttribute(property_attribute::REQUIRED, true);
  }

  auto& config_schema = schema.get("config");
  config_schema.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
  config_schema.setAttribute(property_attribute::REQUIRED, true);
  {
    auto& prop = config_schema.get("conditional");
    prop.setAttribute(property_attribute::TYPE, property_type::BOOL);
    prop.setAttribute(property_attribute::DEFAULT, true);
    prop.setAttribute(property_attribute::DOC, "Enable conditional execution");
    prop.setAttribute(property_attribute::REQUIRED, true);
  }

  {
    auto& inputs = config_schema.get("inputs");
    inputs.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
    inputs.setAttribute(property_attribute::DOC, "Input sources");

    // program
    {
      auto& prop = inputs.get("program");
      prop.setAttribute(property_attribute::TYPE, property_type::STRING);
      prop.setAttribute(property_attribute::DOC, "The composite instruction");
      prop.setAttribute(property_attribute::REQUIRED, true);
    }
    // environment
    {
      auto& prop = inputs.get("environment");
      prop.setAttribute(property_attribute::TYPE, property_type::STRING);
      prop.setAttribute(property_attribute::DOC, "The tesseract environment");
      prop.setAttribute(property_attribute::REQUIRED, true);
    }
  }

  {
    auto& outputs = config_schema.get("outputs");
    outputs.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
    outputs.setAttribute(property_attribute::DOC, "Output sources");
    // program
    {
      auto& prop = outputs.get("program");
      prop.setAttribute(property_attribute::TYPE, property_type::STRING);
      prop.setAttribute(property_attribute::DOC, "The composite instruction");
      prop.setAttribute(property_attribute::REQUIRED, true);
    }
  }

  {
    auto& raster = config_schema.get("raster");
    raster.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
    raster.setAttribute(property_attribute::DOC, "The raster task");
    raster.setAttribute(property_attribute::REQUIRED, true);
    {
      auto& prop = raster.get("task");
      prop.setAttribute(property_attribute::TYPE, property_type::STRING);
      prop.setAttribute(property_attribute::DOC, "The task name");
      prop.setAttribute(property_attribute::REQUIRED, true);
    }
    auto& raster_config = raster.get("config");
    raster_config.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
    raster_config.setAttribute(property_attribute::REQUIRED, true);
    {
      auto& prop = raster_config.get("abort_terminal");
      prop.setAttribute(property_attribute::TYPE, property_type::INT);
      prop.setAttribute(property_attribute::MINIMUM, 0);
      prop.setAttribute(property_attribute::DOC, "The abort terminal");
    }
    {
      auto& prop = raster_config.get("remapping");
      prop.setAttribute(property_attribute::TYPE,
                        property_type::createMap(property_type::STRING, property_type::STRING));
      prop.setAttribute(property_attribute::DOC, "The remapping of input and output keys");
    }
    {
      auto& prop = raster_config.get("indexing");
      prop.setAttribute(property_attribute::TYPE, property_type::createList(property_type::STRING));
      prop.setAttribute(property_attribute::DOC, "The input and output keys to index");
    }
  }

  {
    auto& transition = config_schema.get("transition");
    transition.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
    transition.setAttribute(property_attribute::DOC, "The transition task");
    transition.setAttribute(property_attribute::REQUIRED, true);
    {
      auto& prop = transition.get("task");
      prop.setAttribute(property_attribute::TYPE, property_type::STRING);
      prop.setAttribute(property_attribute::DOC, "The task name");
      prop.setAttribute(property_attribute::REQUIRED, true);
    }
    auto& transition_config = transition.get("config");
    transition_config.setAttribute(property_attribute::TYPE, property_type::CONTAINER);
    transition_config.setAttribute(property_attribute::REQUIRED, true);
    {
      auto& prop = transition_config.get("abort_terminal");
      prop.setAttribute(property_attribute::TYPE, property_type::INT);
      prop.setAttribute(property_attribute::MINIMUM, 0);
      prop.setAttribute(property_attribute::DOC, "The abort terminal");
    }
    {
      auto& prop = transition_config.get("remapping");
      prop.setAttribute(property_attribute::TYPE,
                        property_type::createMap(property_type::STRING, property_type::STRING));
      prop.setAttribute(property_attribute::DOC, "The remapping of input and output keys");
    }
    {
      auto& prop = transition_config.get("indexing");
      prop.setAttribute(property_attribute::TYPE, property_type::createList(property_type::STRING));
      prop.setAttribute(property_attribute::DOC, "The input and output keys to index");
    }
  }

  return schema;
}

TESSERACT_REGISTER_SCHEMA(TaskComposerGraphEdge, getTaskComposerGraphEdgeSchema)

const std::string str = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             program: output_data
                           format_result_as_input: false)";

const std::string str2 = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             program: output_data)";

const std::string str3 = R"(config:
                           conditional: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             programs: output_data)";

const std::string str4 = R"(config:
                           conditional: true
                           should_not_exist: true
                           inputs:
                             program: input_data
                             environment: environment
                             profiles: profiles
                           outputs:
                             program: output_data)";

const std::string graph_str = R"(
class: PipelineTaskFactory
config:
  conditional: true
  nodes:
    StartTask:
      class: StartTaskFactory
      config:
        conditional: false
    DoneTask:
      class: DoneTaskFactory
      config:
        conditional: false
  edges:
    - source: StartTask
      destinations: [DoneTask]
  terminals: [DoneTask])";

std::string raster_only_str = R"(
class: RasterOnlyTaskFactory
config:
  conditional: true
  inputs:
    program: input_data
    environment: environment
  outputs:
    program: output_data
  raster:
    task: CartesianPipeline
    config:
      indexing: [input_data, output_data]
  transition:
    task: CartesianPipeline
    config:
      indexing: [input_data, output_data])";

int testBasic()
{
  // Parse schema from external file (config_schema.yaml)
  PropertyTree schema = buildConfigSchema();

  // Merge schema metadata into config tree
  schema.mergeConfig(YAML::Load(str4));
  std::cout << "Exclude attrubutes:\n" << schema.toYAML() << "\n\n";
  std::cout << "Include attrubutes:\n" << schema.toYAML(false) << "\n\n";

  try
  {
    schema.validate();
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
    return 1;
  }

  bool cond = schema.get("config").get("conditional").getValue().as<bool>();
  std::cout << "conditional = " << std::boolalpha << cond << "\n";
  return 0;
}

int testCustomType()
{
  // Parse schema from external file (config_schema.yaml)
  PropertyTree schema = getTaskComposerGraphSchema();
  std::cout << "Schema:\n" << schema.toYAML(false) << "\n\n";

  // Merge schema metadata into config tree
  schema.mergeConfig(YAML::Load(graph_str));
  std::cout << "Exclude attrubutes:\n" << schema.toYAML() << "\n\n";
  std::cout << "Include attrubutes:\n" << schema.toYAML(false) << "\n\n";

  try
  {
    schema.validate();
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
    return 1;
  }

  return 0;
}

int testRasterOnlyType()
{
  // Parse schema from external file (config_schema.yaml)
  PropertyTree schema = getTaskComposerRasterOnlySchema();
  std::cout << "Schema:\n" << schema.toYAML(false) << "\n\n";

  // Merge schema metadata into config tree
  schema.mergeConfig(YAML::Load(raster_only_str));
  std::cout << "Exclude attrubutes:\n" << schema.toYAML() << "\n\n";
  std::cout << "Include attrubutes:\n" << schema.toYAML(false) << "\n\n";

  try
  {
    schema.validate();
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
    return 1;
  }

  return 0;
}

int main()
{
  // return testCustomType();
  return testRasterOnlyType();
}
