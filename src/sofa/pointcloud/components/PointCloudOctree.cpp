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

#include <sofa/pointcloud/components/PointCloudOctree.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/FileRepository.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <Eigen/Geometry>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/pointcloud/components/utils.h>

namespace sofa::core
{
    template<>
    void registerToFactory<sofa::pointcloud::components::PointCloudOctree>(sofa::core::ObjectFactory* factory){
        factory->registerObjects(sofa::core::ObjectRegistrationData("Store a point cloud.")
                                .add< sofa::pointcloud::components::PointCloudOctree >());
    }
}


void loader(std::atomic<bool>& running, const std::string& name) {
    std::vector<std::string> spinner = {"⠁","⠂","⠄","⡀","⢀","⠠","⠐","⠈"};
    int i = 0;

    std::cout << "\033[?25l"; // hide
    while (running) {
        std::cout << "\rLoading "<< name <<": " << spinner[i % spinner.size()] << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ++i;
    }
    std::cout << "\033[?25h"; // show
}

namespace sofa::pointcloud::components{


    PointCloudOctree::PointCloudOctree() :
        PointCloudGeometry(),
        d_filenames(initData(&d_filenames, "filenames", "List of filenames")),
        d_distances(initData(&d_distances, "distances", "Distances")),
        d_maxSplats(initData(&d_maxSplats, (int)10, "maxSplats", "Max points per leaf")),
        d_showCube(initData(&d_showCube, false, "showCube", "Display bounding boxes")),
        d_showIntersecCube(initData(&d_showIntersecCube, false, "showIntersecCube", "Display bounding boxes")),
        d_cubeColor(initData(&d_cubeColor, sofa::type::RGBAColor(1.0f, 0.0f, 0.0f, 1.0f), "cubeColor", "Color of the bounding boxes")),
        d_intersecCubeColor(initData(&d_intersecCubeColor, sofa::type::RGBAColor(0.0f, 1.0f, 0.0f, 1.0f), "intersecCubeColor", "Color of the bounding boxes")),
        d_cube(initData(&d_cube, Cube(), "cube", "Bounding box of the octree")) { }

    PointCloudOctree::~PointCloudOctree() {
        if (ocTree) {
            delete ocTree;
            ocTree = nullptr;
        }
    }

    void PointCloudOctree::init() {
        if(isComponentStateValid())
            return;

        Inherit1::init();

        const auto& filenames = d_filenames.getValue();
        if(!d_filenames.isSet() || filenames.empty()){
            msg_error(this) << "No filename set";
            d_componentState = core::objectmodel::ComponentState::Invalid;
            return;
        }

        if (d_filenames.getValue().size() == 1 && d_distances.getValue().size() != 1) 
            d_distances.setValue(sofa::type::vector<float>(d_filenames.getValue().size(), 10.0f));

        if (!d_distances.isSet() || d_filenames.getValue().size() != d_distances.getValue().size() ) {
            msg_error(this) << "Distances must be set for each filename";
            d_componentState = core::objectmodel::ComponentState::Invalid;
            return;
        }

        std::atomic<bool> running(true);

        int lod = 0;
        for (const auto& filename : filenames) {
            if(filename.empty()) continue;

            std::atomic<bool> running(true);
            std::thread t(loader, std::ref(running), "LOD " + std::to_string(lod) + " " + filename);

            if( !loadSingleFile(filename, lod) ){
                d_componentState = core::objectmodel::ComponentState::Invalid;
                running = false;
                if(t.joinable()) t.join();
                return;
            }
            running = false;
            if(t.joinable()) t.join();
            
            lod++;
        }

        d_componentState = core::objectmodel::ComponentState::Valid;
    }

    bool PointCloudOctree::loadSingleFile(const std::string& filename, int lod, int max_sh_degree){
        std::string fullname = helper::system::DataRepository.getFile(filename);
        std::ifstream ss(fullname, std::ios::binary);
        std::string fname(filename);

        if (fname.size() < 4 || fname.substr(fname.size() - 4) != ".ply") {
            msg_error() << "File is not a .ply file: " << fname;
            return false;
        }
        if (!ss.is_open()) {
            msg_error() << "Failed to open file: " << fname;
            return false;
        }

        PlyFile file;
        file.parse_header(ss);

        auto request = [&](const std::string& name) {
            return file.request_properties_from_element("vertex", { name }, 1);
        };

        auto x = request("x");
        auto y = request("y");
        auto z = request("z");
        auto opacity = request("opacity");
        auto rot_w = request("rot_0");
        auto rot_x = request("rot_1");
        auto rot_y = request("rot_2");
        auto rot_z = request("rot_3");
        auto scale_0 = request("scale_0");
        auto scale_1 = request("scale_1");
        auto scale_2 = request("scale_2");

        auto f_dc_0 = request("f_dc_0");
        auto f_dc_1 = request("f_dc_1");
        auto f_dc_2 = request("f_dc_2");

        int sh_coeffs = (max_sh_degree + 1) * (max_sh_degree + 1);
        std::vector<std::shared_ptr<PlyData>> f_rest_list;

        for (int i = 0; i < (sh_coeffs - 1); ++i) {
            f_rest_list.push_back(request("f_rest_" + std::to_string(i)));
            f_rest_list.push_back(request("f_rest_" + std::to_string(i + (sh_coeffs - 1))));
            f_rest_list.push_back(request("f_rest_" + std::to_string(i + 2 * (sh_coeffs - 1))));
        }

        file.read(ss);

        int N = x->count;
        auto load_vec = [](std::shared_ptr<PlyData>& pd, int N) -> Eigen::VectorXf {
            return Eigen::Map<Eigen::VectorXf>(reinterpret_cast<float*>(pd->buffer.get()), N);
        };

        Eigen::MatrixXf xyz(N, 3);
        xyz.col(0) = load_vec(x, N);
        xyz.col(1) = load_vec(y, N);
        xyz.col(2) = load_vec(z, N);

        Eigen::MatrixXf rot(N, 4);
        rot.col(0) = load_vec(rot_w, N);
        rot.col(1) = load_vec(rot_x, N);
        rot.col(2) = load_vec(rot_y, N);
        rot.col(3) = load_vec(rot_z, N);

        for (int i = 0; i < N; ++i) rot.row(i).normalize();

        Eigen::MatrixXf scale(N, 3);
        scale.col(0) = load_vec(scale_0, N).array().exp();
        scale.col(1) = load_vec(scale_1, N).array().exp();
        scale.col(2) = load_vec(scale_2, N).array().exp();

        Eigen::MatrixXf opac = load_vec(opacity, N);
        opac = (1.0f / (1.0f + (-opac.array()).exp())).eval();

        Eigen::MatrixXf sh(N, 3 * sh_coeffs);
        sh.col(0) = load_vec(f_dc_0, N);
        sh.col(1) = load_vec(f_dc_1, N);
        sh.col(2) = load_vec(f_dc_2, N);

        int num_extra = 3 * (sh_coeffs - 1);
        for (int i = 0; i < num_extra; ++i) {
            sh.col(3 + i) = load_vec(f_rest_list[i], N);
        }

        int offset = 0;
        if (data == nullptr) {
            data = new GaussianData{ xyz, rot, scale, opac, sh };
        } else {
            offset = data->xyz.rows();
            append(*data, GaussianData{ xyz, rot, scale, opac, sh });
        }

        d_positions.setValue(xyz);
        d_orientations.setValue(rot);
        d_scales.setValue(scale);
        d_opacities.setValue(opac);
        d_sphericalHarmonics.setValue(sh);

        auto indices = helper::getWriteOnlyAccessor(d_indices);
        indices.resize(N);
        for(int i=0;i<N;++i)
        {
            indices[i] = offset + i;
        }


        if (ocTree == nullptr) {
            auto min = sofa::type::Vec3f(std::round(xyz.col(0).minCoeff()) - 1, std::round(xyz.col(1).minCoeff()) - 1, std::round(xyz.col(2).minCoeff()) - 1);
            auto max = sofa::type::Vec3f(std::round(xyz.col(0).maxCoeff()) + 1, std::round(xyz.col(1).maxCoeff()) + 1, std::round(xyz.col(2).maxCoeff()) + 1);
            d_cube.setValue(Cube(min,max));

            ocTree = new QuadTreeNode(&d_distances.getValue());
            ocTree->cube = Cube(min,max);
            ocTree->maxSplats = d_maxSplats.getValue();
        }

        for (int i = 0; i < N; i++) {
            sofa::type::Vec3f pos(xyz(i, 0), xyz(i, 1), xyz(i, 2));

            Splat s = Splat(pos, indices[i]);
            lodPoints[lod].push_back(s);
            if (!ocTree->insertSplat(&lodPoints[lod].back(), lod))
                msg_warning("PointCloudOctree") << "Splat " << (s.indice - offset) << " insertion failed";
        }
        msg_info("PointCloudOctree")  << "LOD " << lod << " has " << N << " splats";
        return true;
    }

    void PointCloudOctree::updateIndices(BaseCamera* camera, float aspect) {
        sofa::type::Vec3 position = camera->getPosition();
        sofa::type::Quat direction = camera->getOrientation();
        auto fov = camera->getFieldOfView();
        auto near = camera->getZNear();
        auto far = camera->getZFar();
        
        if (ocTree == nullptr) {
            msg_error(this) << "No ocTree node";
            return;
        }
        if (position == lastCameraPosition && fov == lastFOV && near == lastNear && far == lastFar)
            return;
        
        
        lastCameraPosition = position;
        lastFOV = fov;
        lastNear = near;
        lastFar = far;
        
        CameraView view(position,
            direction,
            fov,
            near,
            far,
            aspect);
        lastCameraView = view;
            
        std::list<int> indices;
        ocTree->query(view, indices);
        
        auto newIndices = helper::getWriteOnlyAccessor(d_indices);
        int maxSize = indices.size();
        newIndices.resize(maxSize);
        int i = 0;
        for(auto indice : indices)
        {
            newIndices[i++] = indice;
        }
        
        d_componentState = core::objectmodel::ComponentState::Valid;
    }
    
    void PointCloudOctree::drawPlane(const sofa::core::visual::VisualParams* vparams, 
                const Plane& p, 
                const sofa::type::Vec3& origin, 
                const sofa::type::RGBAColor& color) {
        auto* drawer = vparams->drawTool();
        Eigen::Vector3f q = Plane::toEigenF(origin);
        float distanceToPlane = p.normal.dot(q) - p.d;
        Eigen::Vector3f projectedPoint = q - distanceToPlane * p.normal;

        sofa::type::Vec3f pos(projectedPoint.x(), projectedPoint.y(), projectedPoint.z());
        sofa::type::Vec3f normal(p.normal.x(), p.normal.y(), p.normal.z());

        float arrowLength = 1.0f;
        sofa::type::Vec3f arrowEnd = pos + normal * arrowLength;
        drawer->drawArrow(pos, arrowEnd, 0.05f, color);
    }


    void PointCloudOctree::draw(const sofa::core::visual::VisualParams* vparams) {
        if (ocTree == nullptr) return ;
        vparams->drawTool()->saveLastState();
        vparams->drawTool()->disableLighting();
        
        if (d_showCube.getValue())
            ocTree->draw(vparams, d_cubeColor.getValue());

        if (d_showIntersecCube.getValue())
            ocTree->drawIntersec(vparams, d_intersecCubeColor.getValue(), lastCameraView);


        //vparams->drawTool()->drawSphere(sofa::type::Vec3(0,0,0), 0.1f, sofa::type::RGBAColor(1.0f, 1.0f, 1.0f, 1.0f));
        
        vparams->drawTool()->restoreLastState();
    }


}



