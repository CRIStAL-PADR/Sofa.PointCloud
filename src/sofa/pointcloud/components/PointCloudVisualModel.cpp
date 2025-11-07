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
void registerToFactory<sofa::pointcloud::components::PointCloudVisualModel>(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("A point cloud visual model.")
                             .add< sofa::pointcloud::components::PointCloudVisualModel >());
}

}

namespace sofa::pointcloud::components
{

PointCloudVisualModel::PointCloudVisualModel() :
      l_geometry(initLink("geometry", "link to the topology container"))
    , d_indices(initData(&d_indices, "indices", " the indices in the geometry to display"))
    , d_frames(initData(&d_frames, "frames", " set of frame controlling the geometry"))
    , d_frameIndices(initData(&d_frameIndices, "frameIndices", " the indice mapping a surfel to a frame"))
    , d_uniformScale(initData(&d_uniformScale, 1.0f, "uniformScale", " scale factor to apply to whole shape"))
{
    d_frames.setValue({defaulttype::Rigid3Types::Coord{{0.0,0.0,0.0},{0.0,0.0,0.0,1.0}}});
}

PointCloudVisualModel::~PointCloudVisualModel()
{
}

void PointCloudVisualModel::init()
{
    // Track the geometry component state
    addUpdateCallback("update", {&l_geometry->d_componentState}, [this](const sofa::core::DataTracker&){
        return sofa::core::objectmodel::ComponentState::Valid;
    }, {});
}

void PointCloudVisualModel::doInitVisual(const sofa::core::visual::VisualParams* vparams)
{
    SOFA_UNUSED(vparams);

    auto frames = helper::getReadAccessor(d_frames);
    initFrames = d_frames.getValue();
}

}
