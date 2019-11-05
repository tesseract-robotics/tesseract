
#include <tesseract_process_planners/process_definition.h>

namespace tesseract_process_planners
{
ProcessDefinition generateProcessDefinition(const ProcessDefinitionConfig& process_config,
                                            const ProcessSegmentDefinitionConfig& segment_config)
{
  ProcessDefinition process_definition;
  process_definition.start = process_config.start;
  process_definition.segments.reserve(process_config.tool_paths.size());
  process_definition.transitions.reserve(process_config.tool_paths.size() - 1);

  for (size_t i = 0; i < process_config.tool_paths.size(); ++i)
  {
    ProcessSegmentDefinition segment_def;
    if (segment_config.approach != nullptr)
      segment_def.approach = segment_config.approach->generate(process_config.tool_paths[i], process_config);

    if (segment_config.process != nullptr)
      segment_def.process = segment_config.process->generate(process_config.tool_paths[i], process_config);

    if (segment_config.departure != nullptr)
      segment_def.departure = segment_config.departure->generate(process_config.tool_paths[i], process_config);

    process_definition.segments.push_back(segment_def);
  }

  for (size_t i = 0; i < (process_config.tool_paths.size() - 1); ++i)
  {
    ProcessTransitionDefinition transition_def;
    assert(process_config.transition_generator[i] != nullptr);  // transition generators should never be nullptrs
    if (process_config.transition_generator[i] != nullptr)
    {
      if ((process_definition.segments[i].process.front() != nullptr) &&
          (process_definition.segments[i + 1].process.front() != nullptr) &&
          (process_definition.segments[i].process.back() != nullptr) &&
          (process_definition.segments[i + 1].process.back() != nullptr))
      {
        // Connect the appropriate waypoints for the transitions from end
        if (((process_definition.segments[i].departure.empty()) ||
             (process_definition.segments[i].departure.back() == nullptr)) &&
            ((process_definition.segments[i + 1].approach.empty()) ||
             (process_definition.segments[i + 1].approach.front() == nullptr)))
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.back(), process_definition.segments[i + 1].process.front());
        else if (process_definition.segments[i].departure.empty() ||
                 process_definition.segments[i].departure.back() == nullptr)
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.back(), process_definition.segments[i + 1].approach.front());
        else if (process_definition.segments[i + 1].approach.empty() ||
                 process_definition.segments[i + 1].approach.front() == nullptr)
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].departure.back(), process_definition.segments[i + 1].process.front());
        else
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].departure.back(), process_definition.segments[i + 1].approach.front());

        // Connect the appropriate waypoints for the transitions from start
        if (((process_definition.segments[i].approach.empty()) ||
             (process_definition.segments[i].approach.front() == nullptr)) &&
            ((process_definition.segments[i + 1].departure.empty()) ||
             (process_definition.segments[i + 1].departure.back() == nullptr)))
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.front(), process_definition.segments[i + 1].process.back());
        else if (process_definition.segments[i].approach.empty() ||
                 process_definition.segments[i].approach.front() == nullptr)
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.front(), process_definition.segments[i + 1].departure.back());
        else if (process_definition.segments[i + 1].departure.empty() ||
                 process_definition.segments[i + 1].departure.back() == nullptr)
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].approach.front(), process_definition.segments[i + 1].process.back());
        else
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].approach.front(), process_definition.segments[i + 1].departure.back());

        if ((process_definition.segments[i].approach.empty() ||
             (process_definition.segments[i].approach.front() == nullptr)) &&
            (process_definition.segments[i + 1].departure.empty() ||
             (process_definition.segments[i + 1].departure.back() == nullptr)))
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.front(), process_definition.segments[i + 1].process.back());
        else if (process_definition.segments[i].approach.empty() ||
                 process_definition.segments[i].approach.front() == nullptr)
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.front(), process_definition.segments[i + 1].departure.back());
        else if (process_definition.segments[i + 1].departure.empty() ||
                 process_definition.segments[i + 1].departure.back() == nullptr)
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].approach.front(), process_definition.segments[i + 1].process.back());
        else
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].approach.front(), process_definition.segments[i + 1].departure.back());
      }

      process_definition.transitions.push_back(transition_def);
    }
  }

  return process_definition;
}

ProcessDefinition generateProcessDefinition(const ProcessDefinitionConfig& process_config,
                                            const std::vector<ProcessSegmentDefinitionConfig>& segment_config)
{
  ProcessDefinition process_definition;
  process_definition.start = process_config.start;
  process_definition.segments.reserve(process_config.tool_paths.size());
  process_definition.transitions.reserve(process_config.tool_paths.size() - 1);

  for (size_t i = 0; i < process_config.tool_paths.size(); ++i)
  {
    ProcessSegmentDefinition segment_def;
    if (segment_config[i].approach != nullptr)
      segment_def.approach = segment_config[i].approach->generate(process_config.tool_paths[i], process_config);

    if (segment_config[i].process != nullptr)
      segment_def.process = segment_config[i].process->generate(process_config.tool_paths[i], process_config);

    if (segment_config[i].departure != nullptr)
      segment_def.departure = segment_config[i].departure->generate(process_config.tool_paths[i], process_config);

    process_definition.segments.push_back(segment_def);
  }

  for (size_t i = 0; i < (process_config.tool_paths.size() - 1); ++i)
  {
    ProcessTransitionDefinition transition_def;
    assert(process_config.transition_generator[i] != nullptr);  // transition generators should never be nullptrs
    if (process_config.transition_generator[i] != nullptr)
    {
      if ((process_definition.segments[i].process.front() != nullptr) &&
          (process_definition.segments[i + 1].process.front() != nullptr) &&
          (process_definition.segments[i].process.back() != nullptr) &&
          (process_definition.segments[i + 1].process.back() != nullptr))
      {
        // Connect the appropriate waypoints for the transitions from end
        if (((process_definition.segments[i].departure.empty()) ||
             (process_definition.segments[i].departure.back() == nullptr)) &&
            ((process_definition.segments[i + 1].approach.empty()) ||
             (process_definition.segments[i + 1].approach.front() == nullptr)))
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.back(), process_definition.segments[i + 1].process.front());
        else if (process_definition.segments[i].departure.empty() ||
                 process_definition.segments[i].departure.back() == nullptr)
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.back(), process_definition.segments[i + 1].approach.front());
        else if (process_definition.segments[i + 1].approach.empty() ||
                 process_definition.segments[i + 1].approach.front() == nullptr)
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].departure.back(), process_definition.segments[i + 1].process.front());
        else
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].departure.back(), process_definition.segments[i + 1].approach.front());

        // Connect the appropriate waypoints for the transitions from start
        if (((process_definition.segments[i].approach.empty()) ||
             (process_definition.segments[i].approach.front() == nullptr)) &&
            ((process_definition.segments[i + 1].departure.empty()) ||
             (process_definition.segments[i + 1].departure.back() == nullptr)))
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.front(), process_definition.segments[i + 1].process.back());
        else if (process_definition.segments[i].approach.empty() ||
                 process_definition.segments[i].approach.front() == nullptr)
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.front(), process_definition.segments[i + 1].departure.back());
        else if (process_definition.segments[i + 1].departure.empty() ||
                 process_definition.segments[i + 1].departure.back() == nullptr)
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].approach.front(), process_definition.segments[i + 1].process.back());
        else
          transition_def.transition_from_end = process_config.transition_generator[i]->generate(
              process_definition.segments[i].approach.front(), process_definition.segments[i + 1].departure.back());

        if ((process_definition.segments[i].approach.empty() ||
             (process_definition.segments[i].approach.front() == nullptr)) &&
            (process_definition.segments[i + 1].departure.empty() ||
             (process_definition.segments[i + 1].departure.back() == nullptr)))
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.front(), process_definition.segments[i + 1].process.back());
        else if (process_definition.segments[i].approach.empty() ||
                 process_definition.segments[i].approach.front() == nullptr)
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].process.front(), process_definition.segments[i + 1].departure.back());
        else if (process_definition.segments[i + 1].departure.empty() ||
                 process_definition.segments[i + 1].departure.back() == nullptr)
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].approach.front(), process_definition.segments[i + 1].process.back());
        else
          transition_def.transition_from_start = process_config.transition_generator[i]->generate(
              process_definition.segments[i].approach.front(), process_definition.segments[i + 1].departure.back());
      }

      process_definition.transitions.push_back(transition_def);
    }
  }

  return process_definition;
}

}  // namespace tesseract_process_planners
