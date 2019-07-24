# This contains the freespace motion plans that connect both ends of adjacent raster paths.
# Usually only one of them will be used, but this allows a raster path to be omitted on subsequent passes without replanning/approving the freespace motions

tesseract_msgs/ProcessPlanPath from_start
tesseract_msgs/ProcessPlanPath from_end

