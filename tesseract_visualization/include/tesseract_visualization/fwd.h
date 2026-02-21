#ifndef TESSERACT_VISUALIZATION_FWD_H
#define TESSERACT_VISUALIZATION_FWD_H

namespace tesseract::visualization
{
class TrajectoryInterpolator;
class TrajectoryPlayer;
class VisualizationLoader;
class Visualization;
class Marker;
enum class MarkerType : int;

// Markers
class ArrowMarker;
class AxisMarker;
class ContactResultsMarker;
class GeometryMarker;
class ToolpathMarker;

}  // namespace tesseract::visualization
#endif  // TESSERACT_VISUALIZATION_FWD_H
