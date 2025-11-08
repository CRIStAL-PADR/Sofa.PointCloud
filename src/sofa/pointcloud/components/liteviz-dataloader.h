#ifndef __DATALOADER_H__
#define __DATALOADER_H__

#include <string>
#include <vector>
#include <tbb/parallel_sort.h>
#include <Eigen/Dense>
#include <tinyply.h>
#include <fstream>
using namespace tinyply;

struct GaussianData {

    Eigen::MatrixXf xyz;        // N x 3
    Eigen::MatrixXf rot;        // N x 4 (quaternion)
    Eigen::MatrixXf scale;      // N x 3
    Eigen::MatrixXf opacity;    // N x 1
    Eigen::MatrixXf sh;         // N x SH_dim (SH = 3 x ((d+1)^2))

    size_t size() const { return xyz.rows(); }

    int sh_dim() const { return sh.cols(); }

    std::vector<float> flat() const {
        std::vector<float> flat_data;
        flat_data.reserve(size() * (3 + 4 + 3 + 1 + sh_dim()));

        for (int i = 0; i < size(); ++i) {
            for (int j = 0; j < 3; ++j) flat_data.push_back(xyz(i, j));
            for (int j = 0; j < 4; ++j) flat_data.push_back(rot(i, j));
            for (int j = 0; j < 3; ++j) flat_data.push_back(scale(i, j));
            for (int j = 0; j < 1; ++j) flat_data.push_back(opacity(i, j));
            for (int j = 0; j < sh_dim(); ++j) flat_data.push_back(sh(i, j));
        }

        return flat_data;
    }

    static GaussianData load_ply(const char* filename, int max_sh_degree = 3) {
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

        return GaussianData{ xyz, rot, scale, opac, sh };
    }
};

#endif // __DATALOADER_H__
