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

#include <sofa/pointcloud/components/PointCloudInspector.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/ObjectFactory.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <sstream>
namespace sofa::core
{

template<>
void registerToFactory<sofa::pointcloud::components::PointCloudInspector>(sofa::core::ObjectFactory* factory){
    factory->registerObjects(sofa::core::ObjectRegistrationData("Store a point cloud.")
                             .add< sofa::pointcloud::components::PointCloudInspector >());
}

}

namespace sofa::pointcloud::components
{

PointCloudInspector::PointCloudInspector() :
    l_input(initLink("input", "a point cloud container"))
{
}

PointCloudInspector::~PointCloudInspector()
{
}

void PointCloudInspector::draw(const sofa::core::visual::VisualParams* params)
{
    auto dt = params->drawTool();

    dt->saveLastState();
    dt->disableLighting();

    auto xyz = l_input->data->xyz;
    auto sh = l_input->data->sh;

    std::vector<type::Vec3> points;
    std::vector<type::RGBAColor> colors;
    for(int pts = 0; pts < xyz.rows() ; pts++)
    {
        auto v = xyz.row(pts);
        auto s = sh.row(pts).row(0);
        points.emplace_back(type::Vec3{v.x(), v.y(), v.z()});
        colors.emplace_back(type::RGBAColor{s.x(), s.y(), s.z(), s.w()});
    }
    dt->drawPoints(points, 2, colors);

    std::stringstream tmp;
    tmp << "Surfel Couht " << l_input->size() << std::endl;
    dt->writeOverlayText(10,10,20, type::RGBAColor::white(), tmp.str().c_str());

    l_input->computeBBox(nullptr);
    auto bb = l_input->f_bbox.getValue();
    dt->drawBoundingBox(bb.minBBox(), bb.maxBBox(),2);

    dt->restoreLastState();
}

void PointCloudInspector::init()
{
    Inherit1::init();
    d_componentState = core::objectmodel::ComponentState::Valid;
}


}
