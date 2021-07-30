message(STATUS "[VHACD] Generating " ${PROJECT_NAME} "...")
file(
  GLOB
  VHACD_INC_FILES
  "include/tesseract_collision/vhacd/inc/*.h"
  "include/tesseract_collision/vhacd/*.h")
file(GLOB VHACD_INL_FILES "include/tesseract_collision/vhacd/inc/*.inl")
file(GLOB VHACD_CPP_FILES "src/vhacd/*.cpp")
file(GLOB VHACD_C_FILES "src/vhacd/*.c")
file(GLOB VHACD_CL_FILES "include/tesseract_collision/vhacd/cl/*.cl")
source_group(Inc FILES ${VHACD_INC_FILES})
source_group(Inl FILES ${VHACD_INL_FILES})
source_group(Src FILES ${VHACD_CPP_FILES})
source_group(SrcC FILES ${VHACD_C_FILES})
source_group(CL FILES ${VHACD_CL_FILES})

# include_directories(BEFORE ${CMAKE_CURRENT_SOURCE_DIR}/inc)
message(STATUS "[VHACD] \t VHACD_INC_FILES: ${VHACD_INC_FILES}")
message(STATUS "[VHACD] \t VHACD_INL_FILES: ${VHACD_INL_FILES}")
message(STATUS "[VHACD] \t VHACD_CPP_FILES: ${VHACD_CPP_FILES}")
message(STATUS "[VHACD] \t VHACD_C_FILES:   ${VHACD_C_FILES}")
message(STATUS "[VHACD] \t VHACD_CL_FILES:  ${VHACD_CL_FILES}")
