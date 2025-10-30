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

#include <sofa/pointcloud/components/PointCloudTransform.h>
#include <sofa/core/ObjectFactory.h>

#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

namespace sofa::core
{

template<>
void registerToFactory<sofa::pointcloud::components::PointCloudTransform>(sofa::core::ObjectFactory* factory){
    factory->registerObjects(sofa::core::ObjectRegistrationData("Transform a point cloud in a new one.")
                             .add< sofa::pointcloud::components::PointCloudTransform >());
}

}

namespace sofa::pointcloud::components
{

PointCloudTransform::PointCloudTransform() :
     l_input(initLink("input", "a point cloud container"))
    ,l_output(initLink("output", "a second cloud container"))
    ,d_frame(initData(&d_frame, "frame", "a frame to control the geometry"))
    ,d_scale(initData(&d_scale, {1,1,1}, "scale", "a scaling factor"))
{
}

PointCloudTransform::~PointCloudTransform()
{
}

void PointCloudTransform::init()
{
    update();
    d_componentState = core::objectmodel::ComponentState::Valid;

    addUpdateCallback("update", {&d_frame, &d_scale}, [this](const sofa::core::DataTracker&){
        update();
        return sofa::core::objectmodel::ComponentState::Valid;
    }, {&l_output->d_componentState});
}

void PointCloudTransform::update()
{
    std::cout << "UPDATE ... " << std::endl;

    auto source = l_input->data;
    auto destination = l_output->refData;

    auto frame = d_frame.getValue();
    auto center = frame.getCenter();
    auto orientation = frame.getOrientation();
    auto scale = d_scale.getValue()[0];

    auto T = Eigen::Translation<float,3>(center.x(), center.y(), center.z());
    auto R = Eigen::Quaternion<float>(orientation[3], orientation[0], orientation[1], orientation[2]);
    auto S = Eigen::UniformScaling<float>(scale);

    auto transform = T*R*S;

    for(size_t vtxIndex=0;vtxIndex<source->xyz.rows(); ++vtxIndex)
    {
        Eigen::Vector3f worldPosition = ((source->xyz.row(vtxIndex).transpose()));
        destination->xyz.row(vtxIndex) = transform * worldPosition;

        auto tmp = source->rot.row(vtxIndex);
        Eigen::Quaternionf worldOrientation = R * Eigen::Quaternionf{tmp(0), tmp(1), tmp(2), tmp(3)};
        destination->rot.row(vtxIndex)(0) = (worldOrientation).w();
        destination->rot.row(vtxIndex)(1) = (worldOrientation).x();
        destination->rot.row(vtxIndex)(2) = (worldOrientation).y();
        destination->rot.row(vtxIndex)(3) = (worldOrientation).z();

        destination->scale.row(vtxIndex) = source->scale.row(vtxIndex)*scale;
    }

    destination = l_output->data;

    frame = d_frame.getValue();
    center = frame.getCenter();
    orientation = frame.getOrientation();

    T = Eigen::Translation<float,3>(center.x(), center.y(), center.z());
    R = Eigen::Quaternion<float>(orientation[3], orientation[0], orientation[1], orientation[2]);

    transform = T*R*S;

    for(size_t vtxIndex=0;vtxIndex<source->xyz.rows(); ++vtxIndex)
    {
        Eigen::Vector3f worldPosition = ((source->xyz.row(vtxIndex).transpose()));
        destination->xyz.row(vtxIndex) = transform * worldPosition;

        auto tmp = source->rot.row(vtxIndex);
        Eigen::Quaternionf worldOrientation = R * Eigen::Quaternionf{tmp(0), tmp(1), tmp(2), tmp(3)};
        destination->rot.row(vtxIndex)(0) = (worldOrientation).w();
        destination->rot.row(vtxIndex)(1) = (worldOrientation).x();
        destination->rot.row(vtxIndex)(2) = (worldOrientation).y();
        destination->rot.row(vtxIndex)(3) = (worldOrientation).z();

        destination->scale.row(vtxIndex) = source->scale.row(vtxIndex)*scale;
    }
    l_output->updateDataFields();
}


}
