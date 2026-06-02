
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
#include <fstream>
#include <sofa/pointcloud/components/liteviz-dataloader.h>

#include <sofa/pointcloud/components/OctreeNode.h>
#include <sofa/pointcloud/components/PointCloudGeometry.h>
#include <sofa/component/visual/BaseCamera.h>
#include <Eigen/Dense>
#include <sstream>
#include <string>
#include <vector>


namespace sofa::core::objectmodel{
    template<>
    bool Data<Eigen::MatrixXf>::read( const std::string& str );

    template<>
    std::string Data<Eigen::MatrixXf>::getValueString() const;
}

namespace sofa::pointcloud::components{
    using sofa::core::objectmodel::DataFileName;
    using sofa::component::visual::BaseCamera;


    class PointCloudOctree : public PointCloudGeometry  {
        private:
            template<class T>
            using Link = core::objectmodel::SingleLink<PointCloudOctree, T, core::objectmodel::BaseLink::FLAG_STOREPATH>;


        public:
            SOFA_CLASS(PointCloudOctree, PointCloudGeometry);
            
            PointCloudOctree();
            ~PointCloudOctree();

            Data<sofa::type::vector<std::string>> d_filenames;
            Data<sofa::type::vector<float>> d_distances;

            
            Data<int> d_maxSplats;
            Data<Cube> d_cube;
            
            Data<bool> d_showLod;
            Data<sofa::type::vector<sofa::type::RGBAColor>> d_colors;
            Data<bool> d_showCube;
            Data<sofa::type::RGBAColor> d_cubeColor;
            Data<bool> d_showIntersecCube;
            Data<sofa::type::RGBAColor> d_intersecCubeColor;
            
            Data<bool> d_loadingLog;

            QuadTreeNode* ocTree = nullptr;
            std::map<int, std::list<Splat>> lodPoints;
            
            bool loadSingleFile(const std::string& filename, int lod = 0, int max_sh_degree = 3);

            void init() override;


            void drawPlane(const sofa::core::visual::VisualParams* vparams, 
                        const Plane& p, 
                        const sofa::type::Vec3& origin, 
                        const sofa::type::RGBAColor& color);

            void draw(const sofa::core::visual::VisualParams* vparams) override;
            bool updateIndices(BaseCamera* camera, float aspect);
            int getLod(const int index) const;
            
        private:
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> sh_backup;
            bool wasShowLodEnabled = false;

            sofa::type::Vec3 lastCameraPosition;
            double lastFOV;
            double lastNear;
            double lastFar;

            CameraView lastCameraView;
    };

}