#ifndef TESSERACT_PLANNING_OMPL_CONVERSIONS_H
#define TESSERACT_PLANNING_OMPL_CONVERSIONS_H

#include <tesseract_planning/core/macros.h>
TESSERACT_PLANNING_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/PathGeometric.h>
TESSERACT_PLANNING_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>

namespace tesseract_planning
{
tesseract_environment::TrajArray toTrajArray(const ompl::geometric::PathGeometric& path);
}

#endif
