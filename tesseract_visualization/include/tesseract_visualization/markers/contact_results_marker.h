#ifndef TESSERACT_VISUALIZATION_MARKERS_CONTACT_RESULTS_MARKER_H
#define TESSERACT_VISUALIZATION_MARKERS_CONTACT_RESULTS_MARKER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/markers/marker.h>
#include <tesseract_collision/core/types.h>

namespace tesseract_visualization
{
/**
 * @brief A contact results marker
 * @details If margin_fn is provided it takes priority. This can be removed once trajopt_ifopt is fully tested and
 * trajopt_sco and trajopt packages are removed
 */
class ContactResultsMarker : public Marker
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ContactResultsMarker() = default;
  ContactResultsMarker(std::vector<std::string> link_names,
                       tesseract_collision::ContactResultVector dist_results,
                       tesseract_collision::CollisionMarginData margin_data)
    : link_names(std::move(link_names)), dist_results(std::move(dist_results)), margin_data(std::move(margin_data))
  {
  }

  ContactResultsMarker(std::vector<std::string> link_names,
                       tesseract_collision::ContactResultVector dist_results,
                       std::function<double(const std::string&, const std::string&)> margin_fn)
    : link_names(std::move(link_names)), dist_results(std::move(dist_results)), margin_fn(std::move(margin_fn))
  {
  }

  int getType() const override { return static_cast<int>(MarkerType::CONTACT_RESULTS); }

  std::vector<std::string> link_names;
  tesseract_collision::ContactResultVector dist_results;
  tesseract_collision::CollisionMarginData margin_data;
  std::function<double(const std::string&, const std::string&)> margin_fn;
};

}  // namespace tesseract_visualization

#endif  // TESSERACT_VISUALIZATION_MARKERS_CONTACT_RESULTS_MARKER_H
