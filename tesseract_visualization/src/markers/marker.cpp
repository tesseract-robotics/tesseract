
#include <tesseract_visualization/markers/marker.h>

namespace tesseract_visualization
{
void Marker::setParentLink(std::string parent_link) { parent_link_ = std::move(parent_link); }

const std::string& Marker::getParentLink() const { return parent_link_; }

void Marker::setLifetime(const std::chrono::steady_clock::duration& lifetime) { lifetime_ = lifetime; }

std::chrono::steady_clock::duration Marker::getLifetime() const { return lifetime_; }

void Marker::setLayer(int layer) { layer_ = layer; }

int Marker::getLayer() const { return layer_; }

void Marker::setScale(const Eigen::Vector3d& scale) { scale_ = scale; }

const Eigen::Vector3d& Marker::getScale() const { return scale_; }

};  // namespace tesseract_visualization
