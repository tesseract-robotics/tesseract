#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
TESSERACT_IGNORE_WARNINGS_POP

#include "tesseract_planning/ompl/conversions.h"

tesseract::TrajArray tesseract::tesseract_planning::toTrajArray(const ompl::geometric::PathGeometric& path)
{
  const long n_points = static_cast<long>(path.getStateCount());
  const long dof = static_cast<long>(path.getSpaceInformation()->getStateDimension());

  tesseract::TrajArray result(n_points, dof);
  for (long i = 0; i < n_points; ++i)
  {
    const auto& state = path.getState(static_cast<unsigned>(i))->as<ompl::base::RealVectorStateSpace::StateType>();
    for (long j = 0; j < dof; ++j)
    {
      result(i, j) = state->values[j];
    }
  }
  return result;
}
