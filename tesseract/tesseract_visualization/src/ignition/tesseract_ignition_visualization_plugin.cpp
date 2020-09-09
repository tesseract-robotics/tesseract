#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <class_loader/class_loader.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/ignition/tesseract_ignition_visualization.h>

CLASS_LOADER_REGISTER_CLASS(tesseract_visualization::TesseractIgnitionVisualization,
                            tesseract_visualization::Visualization)
