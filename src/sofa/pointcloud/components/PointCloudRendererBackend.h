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
#include <GL/gl.h>
#include <Eigen/Dense>

class BaseGLBuffer
{
public:
    //virtual void init(int ssboID) = 0;
    virtual void cleanup() = 0;
    virtual void map() = 0;
    virtual void unmap() = 0;

    virtual void getValueAsFloats(std::vector<float>& dest) = 0;
    virtual void getValueAsInts(std::vector<int>& dest) = 0;
};

class PointCloudRendererBackend
{
public:
    static bool hasCuda();

    template<class T> static BaseGLBuffer* createBuffer(GLuint ssboID);

    static void sort_float_int(float* keys, int* values, int N);
    static void transform_and_sort_cuda(const Eigen::Matrix4f&, BaseGLBuffer* positions,
                                        BaseGLBuffer* h_keys, BaseGLBuffer* h_values, int N);

    static void transform_and_sort_cpu(const Eigen::Matrix4f& P,
                                  const Eigen::MatrixXf& positions,
                                  std::vector<float>& depths,
                                  std::vector<int>& depth_indices);
};

