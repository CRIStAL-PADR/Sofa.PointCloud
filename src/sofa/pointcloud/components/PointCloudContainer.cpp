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

#define TINYPLY_IMPLEMENTATION 1
#include <sofa/pointcloud/components/PointCloudContainer.h>
#include <sofa/core/ObjectFactory.h>

#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

namespace sofa::core
{

template<>
void registerToFactory<sofa::pointcloud::components::PointCloudContainer>(sofa::core::ObjectFactory* factory){
    factory->registerObjects(sofa::core::ObjectRegistrationData("Store a point cloud.")
                             .add< sofa::pointcloud::components::PointCloudContainer >());
}

}

namespace sofa::pointcloud::components
{

PointCloudContainer::PointCloudContainer() :
    d_filename(initData(&d_filename,"filename","Filename of the object"))
{
}

PointCloudContainer::~PointCloudContainer()
{
}

void PointCloudContainer::init()
{
    std::cout << "INIT " << d_filename.getValue() << std::endl;

    if(!d_filename.isSet() || d_filename.getValue() == ""){
       d_componentState = core::objectmodel::ComponentState::Invalid;
       return;
    }

    load(d_filename.getValue());

    std::cout << "INIT LOAD DONE" << d_filename.getValue() << std::endl;
    std::cout << "  xyz size:" << data->xyz.size() << std::endl;
    std::cout << "  xyz row: " << data->xyz.rows() << std::endl;
    std::cout << "  xyz cols: " << data->xyz.cols() << std::endl;

    d_componentState = core::objectmodel::ComponentState::Valid;
}

void PointCloudContainer::computeBBox(const core::ExecParams* params, bool onlyVisible)
{
    SOFA_UNUSED(params);
    SOFA_UNUSED(onlyVisible);

    type::BoundingBox box;
    auto points = data->xyz;
    for(int i=0;i<points.rows();++i)
    {
        auto point = points.row(i);
        box.include(type::Vec3{point.x(), point.y(), point.z()});
    }
    f_bbox.setValue(box);
    std::cout << "COMPUTE BOUNDING BOX " << box << std::endl;
}

void PointCloudContainer::load(const std::string& filename, int max_sh_degree)
{
    std::ifstream ss(filename, std::ios::binary);
    std::string fname(filename);

    if (fname.size() < 4 || fname.substr(fname.size() - 4) != ".ply") {
        throw std::runtime_error("File is not a .ply file: " + fname);
    }
    if (!ss.is_open()) {
        throw std::runtime_error("Failed to open file: " + fname);
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

    data = new GaussianData{ xyz, rot, scale, opac, sh };
}

size_t PointCloudContainer::size()
{
    return data->xyz.rows();
}

void PointCloudContainer::sort(const Eigen::Matrix4f& P,
                               const type::Vec3& camera_pos,
                               const type::Vec3& camera_dir,
                               std::vector<float>& depths,
                               std::vector<int>& depth_index)
{
    const Eigen::RowVector3f proj_row = P.row(2).head<3>();

//    tbb::parallel_for(static_cast<size_t>(0), depths.size(), [&](auto i){
//        int sorted_idx = depth_index[i];
//        Eigen::Vector3f center = data->xyz.row(sorted_idx).transpose();
//        depths[sorted_idx] = proj_row.dot(center);
//    });

    tbb::parallel_sort(depth_index.begin(), depth_index.end(),
                       [&](int i, int j) {
        return depths[i] < depths[j];
    });
}


}
