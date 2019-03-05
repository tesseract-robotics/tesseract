#ifndef TESSERACT_ROS_PLANNING_CONVERSIONS_H
#define TESSERACT_ROS_PLANNING_CONVERSIONS_H

#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/PathGeometric.h>
TESSERACT_IGNORE_WARNINGS_POP

#include <tesseract_core/basic_env.h>

namespace tesseract
{
namespace tesseract_planning
{
tesseract::TrajArray toTrajArray(const ompl::geometric::PathGeometric& path);
}
}

#endif
