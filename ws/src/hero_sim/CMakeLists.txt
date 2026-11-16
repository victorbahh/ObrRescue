cmake_minimum_required(VERSION 3.8)
project(hero_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf worlds config rviz
  DESTINATION share/${PROJECT_NAME})

# OBR rescue room from the ObrRescue repo root (this package's parent), installed as worlds/obr_rescue.sdf.
# Not copied into this package: with --symlink-install, edits to the root SDF apply immediately.
set(OBR_RESCUE_WORLD "${CMAKE_CURRENT_SOURCE_DIR}/../area_resgate_obr.sdf"
  CACHE FILEPATH "Path to the OBR rescue arena SDF")
if(EXISTS "${OBR_RESCUE_WORLD}")
  install(FILES "${OBR_RESCUE_WORLD}"
    DESTINATION share/${PROJECT_NAME}/worlds RENAME obr_rescue.sdf)
else()
  message(WARNING "OBR rescue world not found at ${OBR_RESCUE_WORLD}; world:=obr_rescue unavailable")
endif()

ament_package()
