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
#include "../extlibs/liteviz/dataloader.h"

namespace sofa::pointcloud::components
{
using sofa::core::objectmodel::DataFileName;
using sofa::core::objectmodel::BaseObject;

class PointCloudContainer : public BaseObject {
public:
    SOFA_CLASS(PointCloudContainer, BaseObject);

    PointCloudContainer();
    ~PointCloudContainer();


    DataFileName d_filename;

    void init() override;
    void computeBBox(const core::ExecParams* /* params */, bool /*onlyVisible*/=false) override;

    void load(const std::string& filename, int max_sh_degree = 3);


    size_t size();

    void sort(const Eigen::Matrix4f& P,
              std::vector<float>& depths,
              std::vector<int> &depth_index);

    GaussianData*    data{nullptr};

 private:
};


}
