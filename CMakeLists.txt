cmake_minimum_required(VERSION 3.12)
project(Sofa.PointCloud LANGUAGES CUDA CXX)

set(SOFA_POINTCLOUD_SRC "src/sofa/pointcloud")

find_package(Sofa.Core REQUIRED)
find_package(Sofa.Component.Visual REQUIRED)
find_package(Sofa.GL REQUIRED)
find_package(SofaPython3 REQUIRED)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(HEADER_FILES
    ${SOFA_POINTCLOUD_SRC}/config.h.in
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudContainer.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudTransform.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudRenderer.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudRendererBackend.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudRendererCUDA.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudVisualModel.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudInspector.h
    ${SOFA_POINTCLOUD_SRC}/components/liteviz-dataloader.h
    # extlibs/liteviz-gs/liteviz/dataloader.h
    # extlibs/liteviz-gs/liteviz/renderer.h
    # extlibs/liteviz-gs/liteviz/shader.h
    # extlibs/liteviz-gs/liteviz/utils.h
    extlibs/tinyply/source/tinyply.h
    # extlibs/diff-gaussian-rasterization/cuda_rasterizer/rasterizer.h
    # extlibs/diff-gaussian-rasterization/cuda_rasterizer/rasterizer_impl.h
    # extlibs/diff-gaussian-rasterization/cuda_rasterizer/forward.h
)

set(SOURCE_FILES
    ${SOFA_POINTCLOUD_SRC}/init.cpp
    ${SOFA_POINTCLOUD_SRC}/fwd.h
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudContainer.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudTransform.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudRenderer.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudRendererBackend.cu
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudVisualModel.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudInspector.cpp
    ${SOFA_POINTCLOUD_SRC}/components/PointCloudRendererCUDA.cpp
)

# Tinyply and liteviz
include_directories("extlibs/tinyply/source")

# TBB
set(TBB_TEST OFF CACHE BOOL "" FORCE)
set(TBB_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
set(TBB_ENABLE_IPO OFF CACHE BOOL "" FORCE)
add_subdirectory("extlibs/tbb")
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
if(TARGET TBB::tbb)
add_custom_command(
    TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:TBB::tbb>
    $<TARGET_FILE_DIR:${PROJECT_NAME}>
    )
    endif()
    
target_link_libraries(${PROJECT_NAME} PUBLIC Sofa.Core Sofa.Component.Visual Sofa.GL SofaPython3::Plugin TBB::tbb)
set_target_properties(${PROJECT_NAME}  PROPERTIES CUDA_ARCHITECTURES "native")

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${Sofa_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR "${PROJECT_NAME}"
    RELOCATABLE "plugins"
)
