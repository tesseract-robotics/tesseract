#ifndef TESSERACT_MOTION_PLANNERS_OMPL_CONVERSIONS_H
#define TESSERACT_MOTION_PLANNERS_OMPL_CONVERSIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/PathGeometric.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

namespace tesseract_motion_planners
{
tesseract_common::TrajArray toTrajArray(const ompl::geometric::PathGeometric& path);
}

#endif
