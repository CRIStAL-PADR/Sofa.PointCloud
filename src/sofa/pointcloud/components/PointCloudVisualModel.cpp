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
    , d_frameIndices(initData(&d_frameIndices, "frameIndices", " the indice mapping each splats to a frame"))
    , d_uniformScale(initData(&d_uniformScale, 1.0f, "uniformScale", " scale factor to apply to whole shape"))
    , d_isStaticModel(initData(&d_isStaticModel, false, "isStatic", "true if the data is fully static" ))
    , d_doInit(initData(&d_doInit, false, "doInit", "true if the data is fully static" ))

{
    d_frames.setValue({defaulttype::Rigid3Types::Coord{{0.0,0.0,0.0},{0.0,0.0,0.0,1.0}}});
}

PointCloudVisualModel::~PointCloudVisualModel()
{
}

void PointCloudVisualModel::init()
{
    Inherit1::init();

    if(!l_geometry)
    {
        msg_error() << "Missing the geometry to render. To remove this message, set the link named 'geometry' so it point to a valid PointCloudContainer ";
        d_componentState = sofa::core::objectmodel::ComponentState::Invalid;
        return;
    }

    if(d_frameIndices.getValue().size()==0 && l_geometry->data)
    {
        auto frames = sofa::helper::getWriteOnlyAccessor(d_frameIndices);
        frames.resize(l_geometry->data->size(), 0);
    }

    // Track the geometry component state
    addUpdateCallback("updateFrameState", {&d_doInit}, [this](const sofa::core::DataTracker&)
    {
        std::cout << "INIT TRANSFORM =======================================" << std::endl;
        if(d_doInit.getValue()){
            d_doInit.setValue(false);
            std::cout << "INIT TRANSFORM ======================================= DONE " << std::endl;
            initTransform();


        }
        return sofa::core::objectmodel::ComponentState::Valid;
    }, {&d_frameIndices, &d_frames});


    // Track the geometry component state
    addUpdateCallback("update", {&l_geometry->d_componentState, &d_frames}, [this](const sofa::core::DataTracker&){
        if(!l_geometry->isComponentStateValid())
        {
            msg_warning() << "The geometry associated with this visual model is in an invalid state";
            return sofa::core::objectmodel::ComponentState::Invalid;
        }

        if(d_frameIndices.getValue().size()!=l_geometry->data->size())
        {
            std::cout << "UPDATE TRANSFORM =======================================" << std::endl;

            auto frames = sofa::helper::getWriteOnlyAccessor(d_frameIndices);
            frames.resize(l_geometry->data->size(), 0);
        }

        return sofa::core::objectmodel::ComponentState::Valid;
    }, {&d_frameIndices});

    initTransform();

    d_componentState = sofa::core::objectmodel::ComponentState::Valid;
}

void PointCloudVisualModel::initTransform()
{
    auto frameIndices = helper::getReadAccessor(d_frameIndices);
    auto frames = helper::getReadAccessor(d_frames);

    auto& positions = l_geometry->data->xyz;
    auto& orientations = l_geometry->data->rot;
    float scale = d_uniformScale.getValue();

    std::cout << "APPLY DEFAULT TRANSFORM " << std::endl;

    referenceFrames.clear();
    localToGlobalFrames.clear();
    for(auto& frame : d_frames.getValue())
    {
        localToGlobalFrames.push_back(frame);
        referenceFrames.push_back(frame);
    };

    // for(size_t vtxIndex=0;vtxIndex<frameIndices.size(); ++vtxIndex)
    // {
    //     int frameIndex = frameIndices[vtxIndex];
    //     auto frameCenter = -frames[frameIndex].getCenter();
    //     auto frameOrientation = (frames[frameIndex].getOrientation()).inverse();

    //     auto T = Eigen::Translation<float,3>(frameCenter.x(), frameCenter.y(), frameCenter.z());
    //     auto R = Eigen::Quaternion<float>(frameOrientation[3], frameOrientation[0], frameOrientation[1], frameOrientation[2]);
    //     auto S = Eigen::UniformScaling<float>(scale);
    //     auto transform = T*R;

    //     Eigen::Vector3f worldPosition = positions.row(vtxIndex).transpose();
    //     positions.row(vtxIndex) = transform * worldPosition;

    //     auto tmp = orientations.row(vtxIndex);
    //     Eigen::Quaternionf worldOrientation = R * Eigen::Quaternionf{tmp(0), tmp(1), tmp(2), tmp(3)};

    //     orientations.row(vtxIndex)(0) = (worldOrientation).w();
    //     orientations.row(vtxIndex)(1) = (worldOrientation).x();
    //     orientations.row(vtxIndex)(2) = (worldOrientation).y();
    //     orientations.row(vtxIndex)(3) = (worldOrientation).z();

    //     //scales.row(vtxIndex) = scales.row(vtxIndex)*scale;
    // }
}

void PointCloudVisualModel::doUpdateVisual(const sofa::core::visual::VisualParams* vparams)
{
    SOFA_UNUSED(vparams);
    d_frameIndices.updateIfDirty();
}

}
