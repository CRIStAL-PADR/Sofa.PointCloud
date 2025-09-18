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
#pragma once

#include <sofa/pointcloud/config.h>
#include <sofa/pointcloud/components/PointCloudContainer.h>
#include <sofa/core/visual/VisualModel.h>
#include <sofa/component/visual/BaseCamera.h>
#include <sofa/gl/GLSLShader.h>
#include <sofa/helper/SelectableItem.h>
#include <ostream>

namespace sofa::pointcloud::components
{
using sofa::core::objectmodel::BaseObject;
using sofa::component::visual::BaseCamera;

// References:
//  - https://huggingface.co/blog/gaussian-splatting
class PointCloudRenderer : public sofa::core::visual::VisualModel {
private:
    template<class T>
    using Link = core::objectmodel::SingleLink<PointCloudRenderer, T, core::objectmodel::BaseLink::FLAG_STOREPATH>;

public:
    SOFA_CLASS(PointCloudRenderer, BaseObject);

    PointCloudRenderer();
    ~PointCloudRenderer() override;

    Link<PointCloudContainer> l_geometry;
    Link<BaseCamera> l_camera;

    Data<helper::OptionsGroup> d_renderMode;
    Data<type::Vec3> d_translation;
    Data<type::Vec3> d_orientation;

    void doInitVisual(const sofa::core::visual::VisualParams* vparams) final;
    void doDrawVisual(const sofa::core::visual::VisualParams* vparams) final;
    void doUpdateVisual(const sofa::core::visual::VisualParams* vparams) final;

private:
    GLuint              _vao;
    GLuint              _vbo;
    GLuint              _ssbo_splat;
    GLuint              _ssbo_index;
    //RenderConfig        _config;

    std::vector<float>  depths;
    std::vector<int>    indices;

    gl::GLSLShader      shader;
};
}
