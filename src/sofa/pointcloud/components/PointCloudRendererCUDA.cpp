/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2025 CNRS, INRIA, USTL, UJF, CNRS, MGH               *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/pointcloud/fwd.h>
#include <sofa/pointcloud/components/PointCloudRendererCUDA.h>
#include <sofa/pointcloud/components/PointCloudVisualModel.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <fstream>
#include <filesystem>
#include <Eigen/Dense>
#include <sofa/helper/system/FileRepository.h>

namespace sofa::core
{

template<>
void registerToFactory<sofa::pointcloud::components::PointCloudRendererCUDA>(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("A point cloud renderer implemented using CUDA.")
                             .add< sofa::pointcloud::components::PointCloudRendererCUDA >());
}

}

namespace sofa::pointcloud::components
{

PointCloudRendererCUDA::PointCloudRendererCUDA() :
    l_targetNode(initLink("targetNode", "Link to the node to start rendering from"))
  , l_camera(initLink("camera", "link to the camera used to render"))
  , d_renderMode(initData(&d_renderMode, "color_sh_0", "renderMode", ""))
{
    helper::OptionsGroup methodOptions{"COLOR_SH_0",
                                       "COLOR_SH_1",
                                       "COLOR_SH_2",
                                       "COLOR_SH_3",
                                       "DEPTH",
                                       "GAUSS_BALL",
                                       "SURFEL"};
    methodOptions.setSelectedItem(0);
    d_renderMode.setValue(methodOptions);
}

PointCloudRendererCUDA::~PointCloudRendererCUDA() {}

void PointCloudRendererCUDA::init()
{
    if(!l_targetNode)
    {
        l_targetNode.set(dynamic_cast<sofa::simulation::Node*>(getContext()));
    }
}

void PointCloudRendererCUDA::doInitVisual(const sofa::core::visual::VisualParams* vparams)
{
}

void PointCloudRendererCUDA::doUpdateVisual(const sofa::core::visual::VisualParams* vparams)
{
    SOFA_UNUSED(vparams);
}

std::function<char*(size_t N)> resizeFunctional(std::vector<float> t) {
    auto lambda = [&t](size_t N) {
        t.resize(N);
        return reinterpret_cast<char*>(t.data());
    };
    return lambda;
}

void PointCloudRendererCUDA::doDrawVisual(const sofa::core::visual::VisualParams* vparams)
{
    //    static void markVisible(
    //        int P,
    //        float* means3D,
    //        float* viewmatrix,
    //        float* projmatrix,
    //        bool* present);

    auto viewport = vparams->viewport();

    Eigen::Matrix4f projmat;
    Eigen::Matrix4f viewmat;
    Eigen::Matrix4d dprojmat;
    Eigen::Matrix4d dviewmat;

    // If there is no camera, then we cannot draw the scene.
    if(!l_camera)
        return;

    // Here we have geometries to draw and a camera that look at it.
    // We first send the camera parameters to the gl rendering backend, then the geometries.
    auto c = l_camera->getPosition();
    Eigen::Vector3f cam_pos {c[0],c[1],c[2]};

    float fov = l_camera->getFieldOfView();
    float tanHalfFov = tan((fov / 180.0 * M_PI) / 2.0f);
    float aspect = static_cast<float>(viewport[2]) / viewport[3];
    Eigen::Vector2f tanxy (aspect * tanHalfFov, tanHalfFov);
    float focal = static_cast<float>(viewport[3]) / (tan((fov / 180.0f * M_PI) / 2.0f) * 2.0f);

    auto mode = d_renderMode.getValue().getSelectedId();

    defaulttype::Rigid3Types::Coord type;

    vparams->drawTool()->pushMatrix();
    float glTransform[16];
    type.writeOpenGlMatrix ( glTransform );
    Eigen::Matrix4f transform {glTransform };

    vparams->getModelViewMatrix(dviewmat.data());
    vparams->getProjectionMatrix(dprojmat.data());

    projmat = dprojmat.cast<float>();
    viewmat = dviewmat.cast<float>();

    std::vector<float> geomBuffer;
    std::vector<float> binningBuffer;
    std::vector<float> imgBuffer;

    std::function<char*(size_t)> geomFunc = resizeFunctional(geomBuffer);
    std::function<char*(size_t)> binningFunc = resizeFunctional(binningBuffer);
    std::function<char*(size_t)> imgFunc = resizeFunctional(imgBuffer);

#if 0
    CudaRasterizer::Rasterizer::forward(
                geomFunc,    //        std::function<char* (size_t)> geometryBuffer,
                binningFunc, //        std::function<char* (size_t)> binningBuffer,
                imgFunc,     //        std::function<char* (size_t)> imageBuffer,
                renderingData.xyz.size(),  // const int P => Nombre de point ?
                0,                         // const int D => degré des harmonique sphériques ?
                0,                         // const int M => nombre max de coéfficien  en lien avec .sh ?
                nullptr,                   // const float* background,
                viewport[2],               // const int width,
                viewport[3],              // const int height,
                renderingData.xyz.,  // const float* means3D,
                renderingData.sh.rows(),   // const flo  at* shs,
                nullptr,                   //        const float* colors_precomp,
                renderingData.opacity.rows(), //       const float* opacities,
                renderingData.scale.rows(),   //        const float* scales,
                1.0,     //        const float scale_modifier,
                renderingData.rot.rows(),  //        const float* rotations,
                nullptr, //        const float* cov3D_precomp,
                viewmat, //        const float* viewmatrix,
                projmat, //        const float* projmatrix,
                cam_pos, //        const float* cam_pos,
                tanxy[0], tanxy[1], //        const float tan_fovx, float tan_fovy,
                false,   //        const bool prefiltered,
                nullptr  //        float* out_color,
                );
#endif
//    static int forward(
//        std::function<char* (size_t)> geometryBuffer,
//        std::function<char* (size_t)> binningBuffer,
//        std::function<char* (size_t)> imageBuffer,
//        const int P, int D, int M,
//        const float* background,
//        const int width, int height,
//        const float* means3D,
//        const float* shs,
//        const float* colors_precomp,
//        const float* opacities,
//        const float* scales,
//        const float scale_modifier,
//        const float* rotations,
//        const float* cov3D_precomp,
//        const float* viewmatrix,
//        const float* projmatrix,
//        const float* cam_pos,
//        const float tan_fovx, float tan_fovy,
//        const bool prefiltered,
//        float* out_color,
//        int* radii = nullptr,
//        bool debug = false);

    //CudaRasterizer::Rasterizer::forward();

}


}
