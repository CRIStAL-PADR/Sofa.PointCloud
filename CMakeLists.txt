cmake_minimum_required(VERSION 3.12)
project(Sofa.PointCloud LANGUAGES CXX)

set(SOFA_POINTCLOUD_SRC "src/sofa/pointcloud")

find_package(Sofa.Core REQUIRED)
find_package(Sofa.Component.Visual REQUIRED)
find_package(Sofa.GL REQUIRED)
find_package(SofaPython3 REQUIRED)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(HEADER_FILES
    ${SOFA_POINTCLOUD_SRC}/config.h.in
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudContainer.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudTransform.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudRenderer.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudVisualModel.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudInspector.h
    extlibs/liteviz/dataloader.h
    extlibs/liteviz/renderer.h
    extlibs/liteviz/shader.h
    extlibs/liteviz/utils.h
)

set(SOURCE_FILES
    ${SOFA_POINTCLOUD_SRC}/init.cpp
    ${SOFA_POINTCLOUD_SRC}/fwd.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudContainer.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudTransform.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudRenderer.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudVisualModel.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudInspector.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.Core Sofa.Component.Visual Sofa.GL SofaPython3::Plugin tbb)

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${Sofa_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src" "extlibs"
    INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
    RELOCATABLE "plugins"
)
