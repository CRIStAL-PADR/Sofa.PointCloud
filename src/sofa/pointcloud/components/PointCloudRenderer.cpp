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
#include <sofa/pointcloud/components/PointCloudRenderer.h>
#include <sofa/pointcloud/components/PointCloudVisualModel.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <fstream>
#include <filesystem>
#include <Eigen/Dense>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/pointcloud/components/PointCloudRendererBackend.h>

namespace sofa::core
{

template<>
void registerToFactory<sofa::pointcloud::components::PointCloudRenderer>(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("A point cloud renderer.")
                             .add< sofa::pointcloud::components::PointCloudRenderer >());
}

}

namespace sofa::pointcloud::components
{

PointCloudRenderer::PointCloudRenderer() :
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

namespace fs = std::filesystem;
std::string readFile(fs::path path)
{
    // Open the stream to 'lock' the file.
    std::ifstream f(path, std::ios::in | std::ios::binary);

    // Obtain the size of the file.
    const auto sz = fs::file_size(path);

    // Create a buffer.
    std::string result(sz, '\0');

    // Read the whole file into the buffer.
    f.read(result.data(), sz);

    return result;
}

PointCloudRenderer::~PointCloudRenderer() {}

void PointCloudRenderer::init()
{
    Inherit1::init();
    if(!l_targetNode)
    {
        l_targetNode.set(dynamic_cast<sofa::simulation::Node*>(getContext()));
    }
}

void PointCloudRenderer::doInitVisual(const sofa::core::visual::VisualParams* vparams)
{
    if (!sofa::gl::GLSLShader::InitGLSL())
    {
        msg_info("BasicShapesGL") << "InitGLSL failed" ;
        return;
    }

    auto vtx = helper::system::DataRepository.getFile("shaders/gaussian_splatting.vert");
    auto fg = helper::system::DataRepository.getFile("shaders/gaussian_splatting.frag");

    auto vertexShader = readFile(vtx);
    auto fragmentShader = readFile(fg);

    shader.SetVertexShaderFileName(vtx);
    shader.SetFragmentShaderFileName(fg);
    shader.InitShaders();

    // Q quad to render a ray-marcher
    std::vector<float> _vertices = {-1.0f,  1.0f, 1.0f,  1.0f, 1.0f, -1.0f, -1.0f, -1.0f};

    glGenVertexArrays(1, &_vao);
    glGenBuffers(1, &_vbo);
    glBindVertexArray(_vao);
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 8, _vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

    glGenBuffers(1, &_ssbo_splat);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, _ssbo_splat);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    glGenBuffers(1, &_ssbo_index);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_index);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, _ssbo_index);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);    

}

void PointCloudRenderer::doUpdateVisual(const sofa::core::visual::VisualParams* vparams)
{
    SOFA_UNUSED(vparams);
}

void clear(GaussianData& data)
{
    data.xyz.resize(0, data.xyz.cols());
    data.sh.resize(0, data.sh.cols());
    data.opacity.resize(0, data.opacity.cols());
    data.rot.resize(0, data.rot.cols());
    data.scale.resize(0, data.scale.cols());
}

void append(Eigen::MatrixXf& dest, const Eigen::MatrixXf& src)
{
    assert(A.cols() == B.cols());

    int oldRows = dest.rows();
    dest.conservativeResize(dest.rows() + src.rows(), src.cols());
    dest.block(oldRows, 0, src.rows(), src.cols()) = src;
}

void append(GaussianData& dest, const GaussianData& src)
{
    append(dest.xyz, src.xyz);
    append(dest.sh, src.sh);
    append(dest.opacity, src.opacity);
    append(dest.scale, src.scale);
    append(dest.rot, src.rot);
}

std::vector<int> range(int maxSize)
{
    std::vector<int> tmp {maxSize};
    for(auto i=0;i<maxSize;++i) tmp.emplace_back(i);
    return tmp;
}

void PointCloudRenderer::sort(const Eigen::Matrix4f& P,
                              const Eigen::MatrixXf& positions,
                              std::vector<float>& depths,
                              std::vector<int>& depth_indices)
{
    const Eigen::RowVector3f proj_row = P.row(2).head<3>();
    for(size_t i=0;i<depth_indices.size(); ++i)
    {
        Eigen::Vector3f center = (positions.row(i).transpose());
        indices[i] = i;
        depths[i] = proj_row.dot(center);
    }

//    tbb::parallel_sort(depth_indices.begin(), depth_indices.end(),
//                       [&](int i, int j) {
//        return depths[i] < depths[j];
//    });

    sort_float_int(depths.data(), depth_indices.data(), depths.size());
    //sort_float_int_to_ssbo(depths.data(), depth_indices.data(), depths.size(), _ssbo_index);
}

void PointCloudRenderer::transform(float scale,
                                   const std::vector<defaulttype::Rigid3Types::Coord>& initFrames,
                                   const std::vector<defaulttype::Rigid3Types::Coord>& frames,
                                   const std::vector<std::vector<int>>& frameIndices,
                                   Eigen::MatrixXf& positions, Eigen::MatrixXf& orientations, Eigen::MatrixXf& scales)
{
    for(size_t frameIndex=0;frameIndex<frameIndices.size();++frameIndex)
    {
        auto frameCenter = frames[frameIndex].getCenter(); // - initFrames[frameIndex].getCenter();
        auto frameOrientation = frames[frameIndex].getOrientation();// - initFrames[frameIndex].getOrientation();

        auto T = Eigen::Translation<float,3>(frameCenter.x(), frameCenter.y(), frameCenter.z());
        auto R = Eigen::Quaternion<float>(frameOrientation[3], frameOrientation[0], frameOrientation[1], frameOrientation[2]);
        auto S = Eigen::UniformScaling<float>(scale);

        auto transform = T*R;
        for(size_t i=0;i<frameIndices[frameIndex].size(); ++i)
        {
            auto vtxIndex = frameIndices[frameIndex][i];

            Eigen::Vector3f worldPosition = positions.row(vtxIndex).transpose();
            positions.row(vtxIndex) = transform * worldPosition;

            auto tmp = orientations.row(vtxIndex);
            Eigen::Quaternionf worldOrientation = R * Eigen::Quaternionf{tmp(0), tmp(1), tmp(2), tmp(3)};

            orientations.row(vtxIndex)(0) = (worldOrientation).w();
            orientations.row(vtxIndex)(1) = (worldOrientation).x();
            orientations.row(vtxIndex)(2) = (worldOrientation).y();
            orientations.row(vtxIndex)(3) = (worldOrientation).z();

            scales.row(vtxIndex) = scales.row(vtxIndex)*scale;
        }
    }
}

void PointCloudRenderer::doDrawVisual(const sofa::core::visual::VisualParams* vparams)
{
    auto viewport = vparams->viewport();

    Eigen::Matrix4f projmat;
    Eigen::Matrix4f viewmat;
    Eigen::Matrix4d dprojmat;
    Eigen::Matrix4d dviewmat;

    // If there is no camera, then we cannot draw the scene.
    if(!l_camera)
        return;

    // Aggregate the geometries by traversing the scene tree
    auto visualModels = l_targetNode->getTreeObjects<sofa::pointcloud::components::PointCloudVisualModel>();
    msg_info() << "Found " << visualModels.size() << " gaussian splats visual models.";

    static bool isFirstRun = true;
    static bool hasStaticData=false;

    if(!hasStaticData)
    {
        clear(renderingData);

        for(auto visual : visualModels)
        {
            visual->updateVisual(vparams);

            if(!visual->isComponentStateValid()){
                continue;
            }

            if(!visual->l_geometry->data){
                continue;
            }
            int offset = renderingData.xyz.rows();

            //if(isFirstRun)
            //{
            //    visual->initTransform();
            //}

            if(visual->d_isStaticModel.getValue())
                hasStaticData=true;

            // First build the reference geometry
            append(renderingData.xyz, visual->l_geometry->data->xyz);
            append(renderingData.sh, visual->l_geometry->data->sh);
            append(renderingData.opacity, visual->l_geometry->data->opacity);
            append(renderingData.scale, visual->l_geometry->data->scale);
            append(renderingData.rot, visual->l_geometry->data->rot);

            auto scale = visual->d_uniformScale.getValue();
            auto initFrames = visual->initFrames;
            auto frames = helper::getReadAccessor(visual->d_frames);
            auto frameIndices = helper::getReadAccessor(visual->d_frameIndices);

            std::vector<std::vector<int>> frameMap{frames.size()};
            for(size_t i=0;i<frameIndices.size();++i)
            {
                frameMap[frameIndices[i]].push_back(offset+i);
            }

            //visual->doUpdateVisual(sofa::core::visual::VisualParams::defaultInstance());

            // Now we need to apply the transformation
            this->transform(scale,
                            initFrames,
                            frames, frameMap, renderingData.xyz, renderingData.rot, renderingData.scale);
            msg_info() << "Batching a new data set " << visual->getPathName() << " with frames " << frames.size();
        }
    }
    msg_info() << "  total number of splats to render: " << renderingData.size() << " splats ";

    isFirstRun=false;
    // In case there is not rendering data, then just exit
    if(renderingData.size()==0)
        return;

    // Here we have geometries to draw and a camera that look at it.
    // We first send the camera parameters to the gl rendering backend, then the geometries.
    auto c = l_camera->getPosition();
    Eigen::Vector3f cam_pos {c[0],c[1],c[2]};

    vparams->getModelViewMatrix(dviewmat.data());
    vparams->getProjectionMatrix(dprojmat.data());

    projmat = dprojmat.cast<float>();
    viewmat = dviewmat.cast<float>();

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


    shader.TurnOn();

    float scale_modifier = 1;
    int max_sh_dim = 48;

    shader.SetMatrix4(shader.GetVariable("projmat"), 1, GL_FALSE, projmat.data());
    shader.SetMatrix4(shader.GetVariable("viewmat"), 1, GL_FALSE, viewmat.data());
    shader.SetFloatVector3(shader.GetVariable("cam_pos"), 1, cam_pos.data());
    shader.SetFloatVector2(shader.GetVariable("tanxy"), 1, tanxy.data());
    shader.SetFloat(shader.GetVariable("focal"), focal);
    shader.SetInt(shader.GetVariable("max_sh_dim"), max_sh_dim);
    shader.SetInt(shader.GetVariable("render_mod"), mode);
    shader.SetFloat(shader.GetVariable("scale_modifier"), scale_modifier);

    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    indices = range(renderingData.size());
    static bool isInited = false;
    if(!isInited || !hasStaticData){
        isInited = true;

        std::vector<float> flatData = renderingData.flat();
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat);
        glBufferData(GL_SHADER_STORAGE_BUFFER, flatData.size() * sizeof(float), flatData.data(), GL_STATIC_DRAW);
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, _ssbo_splat);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }

    indices = range(renderingData.size());
    depths.resize(indices.size());

    sort(viewmat, renderingData.xyz, depths, indices);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_index);
    glBufferData(GL_SHADER_STORAGE_BUFFER, indices.size() * sizeof(int), indices.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, _ssbo_index);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    glBindVertexArray(_vao);
    glEnableVertexAttribArray(0);

    glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 4, indices.size());

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDisableVertexAttribArray(0);
    glBindVertexArray(0);

    shader.TurnOff();

    glEnable(GL_CULL_FACE);

    vparams->drawTool()->popMatrix();
}


}
