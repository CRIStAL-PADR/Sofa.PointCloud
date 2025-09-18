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
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <fstream>
#include <filesystem>
#include <Eigen/Dense>

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
      l_geometry(initLink("geometry", "link to the topology container"))
    , l_camera(initLink("camera", "link to the camera used to render"))
    , d_renderMode(initData(&d_renderMode, "color_sh_0", "renderMode", ""))
    , d_translation(initData(&d_translation, {0.0,0.0,0.0}, "translation", ""))
    , d_orientation(initData(&d_orientation, {0.0,0.0,0.0}, "orientation", ""))
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

PointCloudRenderer::~PointCloudRenderer()
{

}

void PointCloudRenderer::doInitVisual(const sofa::core::visual::VisualParams* vparams)
{
    std::cout << "DO INIT VISUAL " << std::endl;

    if (!sofa::gl::GLSLShader::InitGLSL())
    {
        msg_info("BasicShapesGL") << "InitGLSL failed" ;
        return;
    }

    auto vertexShader = readFile("./assets/shaders/gaussian_splatting.vert");
    auto fragmentShader = readFile("./assets/shaders/gaussian_splatting.frag");

    std::cout << "HELLO " << fragmentShader << std::endl;;

    shader.SetVertexShaderFileName("./assets/shaders/gaussian_splatting.vert");
    shader.SetFragmentShaderFileName("./assets/shaders/gaussian_splatting.frag");
    shader.InitShaders();

    // Q quad to render a ray-marcher
    std::vector<float> _vertices = {-1.0f,  1.0f, 1.0f,  1.0f, 1.0f, -1.0f, -1.0f, -1.0f};

    //return;
    glGenVertexArrays(1, &_vao);
    glGenBuffers(1, &_vbo);
    glBindVertexArray(_vao);
    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 8, _vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, nullptr);

    if(l_geometry->isComponentStateInvalid())
        return;
    std::vector<float> flatData = l_geometry->data->flat();
    glGenBuffers(1, &_ssbo_splat);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_splat);
    glBufferData(GL_SHADER_STORAGE_BUFFER, flatData.size() * sizeof(float), flatData.data(), GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, _ssbo_splat);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    depths.resize(l_geometry->size());
    indices.resize(l_geometry->size());
    for(unsigned int i=0;i<depths.size();++i)
    {
        depths[i] = 0;
        indices[i] = i;
    }
    l_geometry->sort(Eigen::Matrix4f::Identity(),depths,indices);
    glGenBuffers(1, &_ssbo_index);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_index);
    glBufferData(GL_SHADER_STORAGE_BUFFER, indices.size() * sizeof(int), indices.data(), GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, _ssbo_index);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void PointCloudRenderer::doDrawVisual(const sofa::core::visual::VisualParams* vparams)
{
    std::cout << "DOD DRAW VISUAL " << std::endl;
    auto viewport = vparams->viewport();

    Eigen::Matrix4f projmat;
    Eigen::Matrix4f viewmat;
    Eigen::Matrix4d dprojmat;
    Eigen::Matrix4d dviewmat;

    std::cout << "DOD DRAW VISUAL 2" << std::endl;
    if(!l_camera)
        return;

    auto c = l_camera->getPosition();
    Eigen::Vector3f cam_pos {c[0],c[1],c[2]};

    std::cout << "DOD DRAW VISUAL 3" << std::endl;
    float fov = l_camera->getFieldOfView();
    float tanHalfFov = tan((fov / 180.0 * M_PI) / 2.0f);
    float aspect = static_cast<float>(viewport[2]) / viewport[3];
    Eigen::Vector2f tanxy (aspect * tanHalfFov, tanHalfFov);
    float focal = static_cast<float>(viewport[3]) / (tan((fov / 180.0f * M_PI) / 2.0f) * 2.0f);

    std::cout << "DOD DRAW VISUAL 4" << std::endl;
    //_config = RenderConfig();
    int num_primitives = l_geometry->data->size();
    float sh_dim = l_geometry->data->sh_dim();
    auto mode = d_renderMode.getValue().getSelectedId();

    defaulttype::Rigid3Types::Coord type;
    type.getCenter() = d_translation.getValue();

    auto t = d_orientation.getValue();
    type.getOrientation() = type::Quatf::fromEuler(t[0],t[1],t[2]);

    vparams->drawTool()->pushMatrix();
    float glTransform[16];
    type.writeOpenGlMatrix ( glTransform );
    Eigen::Matrix4f transform {glTransform };
    std::cout << "YO LO data " << d_translation.getValue() << std::endl;
    std::cout << "YO LO type " << type << std::endl;
    std::cout << "YO LO tran " << glTransform[0] << ", " << glTransform[1] << ", " << glTransform[2] << std::endl;

    vparams->getModelViewMatrix(dviewmat.data());
    vparams->getProjectionMatrix(dprojmat.data());

    projmat = dprojmat.cast<float>();
    viewmat = dviewmat.cast<float>();

    viewmat = transform * viewmat ;
    std::cout << "YO LO transform " << transform << std::endl;
    std::cout << "YO LO 2 " << viewmat << std::endl;

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

    l_geometry->sort(viewmat, depths, indices);

    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, _ssbo_index);
    glBufferData(GL_SHADER_STORAGE_BUFFER, indices.size() * sizeof(int), indices.data(), GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, _ssbo_index);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);

    glBindBuffer(GL_ARRAY_BUFFER, _vbo);
    glBindVertexArray(_vao);
    glEnableVertexAttribArray(0);

    glDrawArraysInstanced(GL_TRIANGLE_FAN, 0, 4, static_cast<int>(l_geometry->data->size()));
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glDisableVertexAttribArray(0);

    shader.TurnOff();

    glEnable(GL_CULL_FACE);

    vparams->drawTool()->popMatrix();

    //glEn(GL_BLEND);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

}

void PointCloudRenderer::doUpdateVisual(const sofa::core::visual::VisualParams* vparams)
{
    std::cout << "DO UPDATE VISUAL " << std::endl;
}

}
