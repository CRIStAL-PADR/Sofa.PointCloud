#include <sofa/pointcloud/components/PointCloudRendererBackend.h>

#include <cub/cub.cuh>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <cuda_gl_interop.h>

#define CUDA_CHECK(call) do { \
  cudaError_t err = call; \
  if (err != cudaSuccess) { \
    fprintf(stderr, "CUDA Error %s:%d: %s\n", __FILE__, __LINE__, cudaGetErrorString(err)); \
  } \
} while(0)

template<class T>
class CudaGLBuffer : public BaseGLBuffer
{
public:
    GLuint glSSBO {0};
    cudaGraphicsResource* cudaBuffer{nullptr};

    CUdaGLBuffer(GLuint ssboID)
    {
        glSSBO = ssboID;
    }

    void cleanup() override
    {
        CUDA_CHECK(cudaGraphicsUnregisterResource(cudaBuffer));
    }

    void map() override
    {
        if(cudaBuffer==nullptr)
            CUDA_CHECK(cudaGraphicsGLRegisterBuffer(&cudaBuffer, glSSBO, cudaGraphicsRegisterFlagsWriteDiscard));

        CUDA_CHECK(cudaGraphicsMapResources(1, &cudaBuffer));
        auto [_, size] = getSSBOAndSize();

        std::cout << "MAP FOR " << cudaBuffer << " ," << glSSBO << " size=" << size << std::endl;
     }

    void unmap() override
    {
        CUDA_CHECK(cudaGraphicsUnmapResources(1, &cudaBuffer));
    }

    // Returns the ssbo id & size
    std::tuple<T*, size_t> getSSBOAndSize()
    {
        std::cout << "[DEBUG] glSSBO=" << glSSBO
                  << " cudaBuffer=" << cudaBuffer << std::endl;

        int* ssboPtr = nullptr;
        size_t ssboSize = 0;
        CUDA_CHECK(cudaGraphicsResourceGetMappedPointer((void**)&ssboPtr, &ssboSize, cudaBuffer));

        std::cout << "[DEBUG] ssboPtr=" << ssboPtr
                  << " ssboSize=" << ssboSize << std::endl;

        return {reinterpret_cast<T*>(ssboPtr), ssboSize};
    }

    void getValueAsFloats(std::vector<float>& values)
    {
        auto [ssbo, size] = getSSBOAndSize();
        values.resize(size);
        cudaMemcpy(values.data(), ssbo, size * sizeof(float), cudaMemcpyDeviceToHost);
    }

    void getValueAsInts(std::vector<int>& values)
    {
        auto [ssbo, size] = getSSBOAndSize();
        values.resize(size);
        cudaMemcpy(values.data(), ssbo, size * sizeof(int), cudaMemcpyDeviceToHost);
    }
};


template<> BaseGLBuffer* PointCloudRendererBackend::createBuffer<int>(GLuint ssboID)
{
    return new CudaGLBuffer<int>(ssboID);
}

template<> BaseGLBuffer* PointCloudRendererBackend::createBuffer<float>(GLuint ssboID)
{
    return new CudaGLBuffer<float>(ssboID);
}

void PointCloudRendererBackend::sort_float_int(float* h_keys, int* h_values, int N)
{
    // Copie sur GPU
    thrust::device_vector<float> d_keys(h_keys, h_keys + N);
    thrust::device_vector<int>   d_values(h_values, h_values + N);

    // Tri : keys → triées, values → réarrangées dans le même ordre
    thrust::sort_by_key(d_keys.begin(), d_keys.end(), d_values.begin());

    // Copie vers CPU
    thrust::copy(d_keys.begin(), d_keys.end(), h_keys);
    thrust::copy(d_values.begin(), d_values.end(), h_values);

}

void project_to_z(const Eigen::Matrix4f&, CudaGLBuffer<float>& positions, CudaGLBuffer<float>& z, const int N);
void sort_float_int(CudaGLBuffer<float>& values, CudaGLBuffer<int>& indices);

void PointCloudRendererBackend::transform_and_sort_cuda(const Eigen::Matrix4f& mvp,
                        BaseGLBuffer* positions_, BaseGLBuffer* depths_, BaseGLBuffer* indices_, int N)
{
    std::cout << "MAIS OUI " << std::endl;
    auto positions = dynamic_cast<CudaGLBuffer<float>*>(positions_);
    auto depths = dynamic_cast<CudaGLBuffer<float>*>(depths_);
    auto indices = dynamic_cast<CudaGLBuffer<int>*>(indices_);
    assert(positions!=nullptr);
    assert(depths!=nullptr);
    assert(indices!=nullptr);

    glFinish();

    positions->map();
    depths->map();
    indices->map();

    auto [values_ptr, values_size] = positions->getSSBOAndSize();
    auto [indices_ptr, indices_size] = indices->getSSBOAndSize();

    assert(values_size != N);
    assert(indices_size != N);

    thrust::device_ptr<float> d_keys(values_ptr);
    thrust::device_ptr<int>   d_vals(indices_ptr);

    std::cout << "BEFORE DEPTH" << std::endl;
    project_to_z(mvp, *positions, *depths, N);

    std::vector<float> depths_cpu;
    depths->getValueAsFloats(depths_cpu);
    std::stringstream tmp;
    tmp << "Depths: ";
    for(unsigned int i =0;i<20;i++){
        tmp << " " << depths_cpu[i];
    }
    std::cout << tmp.str() << std::endl;

    thrust::sort_by_key(d_keys, d_keys+values_size, d_vals);

    std::vector<int> indices_cpu;
    indices->getValueAsInts(indices_cpu);
    std::stringstream tmp2;
    tmp2 << "Indices: ";
    for(unsigned int i =0;i<20;i++){
        tmp2 << " " << indices_cpu[i];
    }
    std::cout << tmp2.str() << std::endl;

    positions->unmap();
    depths->unmap();
    indices->unmap();
}


#include <cuda_runtime.h>

struct vec3 { float x, y, z; };
struct mat4 { float m[16]; }; // row-major

__device__ float compute_depth(const vec3& pos, const float* mvp)
{
    const mat4* proj = (const mat4*)mvp;

    // Transforme le vertex en vec4
    float x = pos.x;
    float y = pos.y;
    float z = pos.z;

    // Multiplication matrice 4x4
    float clip_z = proj->m[2]*x + proj->m[6]*y + proj->m[10]*z + proj->m[14];
    float clip_w = proj->m[3]*x + proj->m[7]*y + proj->m[11]*z + proj->m[15];

    // Depth normalisée (NDC z)
    return 0.0; //clip_z / clip_w;
}

__global__ void compute_depth_kernel(
    const vec3* positions,
    float* depths,
    const int N,
    const float* proj
)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= N) return;

    depths[idx] = -1.0; //compute_depth(positions[idx], proj);
}

void project_to_z(const Eigen::Matrix4f& mvp, CudaGLBuffer<float>& positions, CudaGLBuffer<float>& depths, int N)
{
    float* d_proj;
    cudaMalloc(&d_proj, sizeof(float) * 16);
    cudaMemcpy(d_proj, mvp.data(), sizeof(float) * 16, cudaMemcpyHostToDevice);

    auto [posPtr, posSize] = positions.getSSBOAndSize();
    auto [depthPtr, depthSize] = depths.getSSBOAndSize();

    // 2. Lancer le kernel
    int blockSize = 256;
    int gridSize = (N + blockSize - 1) / blockSize;
    compute_depth_kernel<<<gridSize, blockSize>>>((const vec3*)posPtr, (float*)depthPtr, N, (const float*)d_proj);

    cudaDeviceSynchronize();
    cudaFree(d_proj);
}

bool PointCloudRendererBackend::hasCuda()
{
    return true;
}

void PointCloudRendererBackend::transform_and_sort_cpu(const Eigen::Matrix4f& P,
                                                const Eigen::MatrixXf& positions,
                                                std::vector<float>& depths,
                                                std::vector<int>& indices)
{
    const Eigen::RowVector3f proj_row = P.row(2).head<3>();
    for(size_t i=0;i<indices.size(); ++i)
    {
        Eigen::Vector3f center = (positions.row(i).transpose());
        indices[i] = i;
        depths[i] = proj_row.dot(center);
    }

    std::sort(indices.begin(), indices.end(),
                       [&](int i, int j) {
        return depths[i] < depths[j];
    });
}
