cmake_minimum_required(VERSION 3.12)
project(Sofa.PointCloud LANGUAGES CXX)

set(SOFA_POINTCLOUD_SRC "src/sofa/pointcloud")

find_package(Sofa.Core REQUIRED)
find_package(SofaPython3 REQUIRED)

set(HEADER_FILES
    ${SOFA_POINTCLOUD_SRC}/config.h.in
)

set(SOURCE_FILES
    ${SOFA_POINTCLOUD_SRC}/init.cpp
)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.Core SofaPython3::Plugin)

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${Sofa_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
    RELOCATABLE "plugins"
)
