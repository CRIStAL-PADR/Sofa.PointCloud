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
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/pointcloud/components/PointCloudGeometry.h>

namespace sofa::pointcloud::components
{
using sofa::core::objectmodel::DataFileName;
using sofa::core::objectmodel::BaseObject;

class PointCloudTransform : public BaseObject {
public:
    SOFA_CLASS(PointCloudTransform, BaseObject);

    PointCloudTransform();
    ~PointCloudTransform();

    void init() override;

    SingleLink<PointCloudTransform, PointCloudGeometry, sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK> l_input;
    SingleLink<PointCloudTransform, PointCloudGeometry, sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK> l_output;

    void clear(GaussianData& data);
    void append(Eigen::MatrixXf& dest, const Eigen::MatrixXf& src);
    void append(Eigen::Matrix<float, Eigen::Dynamic, 1>& dest,
                                     const Eigen::Matrix<float, Eigen::Dynamic, 1>& src);
    template<unsigned int Size>
    void append(Eigen::Matrix<float, Eigen::Dynamic, Size, Eigen::RowMajor>& dest,
                                     const Eigen::Matrix<float, Eigen::Dynamic, Size, Eigen::RowMajor>& src);
    void append(GaussianData& dest, const GaussianData& src);

private:
    Data<type::vector<defaulttype::Rigid3Types::Coord>> d_frames;
    Data<type::vector<unsigned int >> d_frameIndices;
    Data<type::vector<SReal>> d_scales;
    void update();
};


}
