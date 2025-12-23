#include <sofa/pointcloud/components/PointCloudRendererBackend.h>

#include <cub/cub.cuh>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <cub/device/device_radix_sort.cuh>
#include <cuda_fp16.h>
#include <cuda_gl_interop.h>
#include <tbb/tbb.h>
#include <algorithm>

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

    CudaGLBuffer(GLuint ssboID)
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
    }

    void unmap() override
    {
        CUDA_CHECK(cudaGraphicsUnmapResources(1, &cudaBuffer));
    }

    // Returns the ssbo id & size in byte
    std::tuple<T*, size_t> getSSBOAndSize()
    {
        T* ssboPtr = nullptr;
        size_t ssboSize = 0;
        CUDA_CHECK(cudaGraphicsResourceGetMappedPointer((void**)&ssboPtr, &ssboSize, cudaBuffer));

        return {ssboPtr, ssboSize};
    }

    void getValueAsFloats(std::vector<float>& values)
    {
        auto [ssbo, size] = getSSBOAndSize();
        values.resize(size/sizeof(float));
        CUDA_CHECK(cudaMemcpy(values.data(), ssbo, size, cudaMemcpyDeviceToHost));
    }

    void getValueAsInts(std::vector<int>& values)
    {
        auto [ssbo, size] = getSSBOAndSize();
        values.resize(size/sizeof(int));
        CUDA_CHECK(cudaMemcpy(values.data(), ssbo, size, cudaMemcpyDeviceToHost));
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

void project_to_z(const Eigen::Matrix4f&, CudaGLBuffer<float>& positions,
                  short* depthPtr, int* indicesPtr, const int N);
void sort_float_int(CudaGLBuffer<float>& values, CudaGLBuffer<int>& indices);

class RadixSortBuffer
{
public:
    int N {0};
    cub::DoubleBuffer<short> keys;
    cub::DoubleBuffer<int>   indices;
    void* temporary_ptr{nullptr};
    size_t temporary_size = 0;

    void reset(int N_)
    {
        std::cout << "RESET CUDA BUFFER " << std::endl;
        N = N_;

        short* d_depths_out;
        int*   d_indices_out;
        cudaMalloc(&d_depths_out,  N * sizeof(short));
        cudaMalloc(&d_indices_out, N * sizeof(int));

        short* d_depths_out2;
        int*   d_indices_out2;
        cudaMalloc(&d_depths_out2,  N * sizeof(short));
        cudaMalloc(&d_indices_out2, N * sizeof(int));

        keys = cub::DoubleBuffer<short>(d_depths_out, d_depths_out2);
        indices = cub::DoubleBuffer<int>(d_indices_out, d_indices_out2);

        // Taille du buffer temporaire
        cub::DeviceRadixSort::SortPairs(
                    temporary_ptr, temporary_size,
                    keys, indices,
                    N);

        // Allocation
        cudaMalloc(&temporary_ptr, temporary_size);
    }

    ~RadixSortBuffer(){
        cudaFree(temporary_ptr);
        cudaFree(keys.d_buffers[0]);
        cudaFree(keys.d_buffers[1]);
        cudaFree(indices.d_buffers[0]);
        cudaFree(indices.d_buffers[1]);
    }
};

void PointCloudRendererBackend::transform_and_sort_cuda(const Eigen::Matrix4f& mvp,
                                                        BaseGLBuffer* positions_, BaseGLBuffer* depths_, BaseGLBuffer* indices_, int N)
{
    static RadixSortBuffer buffers;

    auto positions = dynamic_cast<CudaGLBuffer<float>*>(positions_);
    //auto depths = dynamic_cast<CudaGLBuffer<float>*>(depths_);
    auto indices = dynamic_cast<CudaGLBuffer<int>*>(indices_);

    assert(positions!=nullptr);
    //assert(depths!=nullptr);
    assert(indices!=nullptr);

    if(buffers.N < N)
    {
        buffers.reset(N);
    }

    //glFinish();

    positions->map();
    indices->map();

    buffers.keys.Alternate();
    buffers.indices.Alternate();

    project_to_z(mvp, *positions,
                 buffers.keys.Current(), buffers.indices.Current(), N);

    auto [indices_ptr, indices_size] = indices->getSSBOAndSize();

    assert(depths_size/sizeof(short) == N);
    assert(indices_size/sizeof(int) == N);

    // Tri effectif
    cub::DeviceRadixSort::SortPairs(
                buffers.temporary_ptr, buffers.temporary_size,
                buffers.keys, buffers.indices,
                buffers.N);

    cudaMemcpy(indices_ptr, buffers.indices.Current(), N*sizeof(int),  cudaMemcpyDeviceToDevice);

    positions->unmap();
    //depths->unmap();
    indices->unmap();
}


#include <cuda_runtime.h>

struct vec3 { float x, y, z; };
struct mat4 { float m[16]; }; // row-major

__device__ short compute_depth(const vec3& pos, const float* mvp)
{
    const mat4* proj = (const mat4*)mvp;

    // Transforme le vertex en vec4
    float x = pos.x;
    float y = pos.y;
    float z = pos.z;

    // ligne 2 de P, colonnes 0..2
    float proj0 = proj->m[0*4 + 2];
    float proj1 = proj->m[1*4 + 2];
    float proj2 = proj->m[2*4 + 2];

    // produit depth scalaire
    return ((proj0 * x + proj1 * y + proj2 * z)/200)*65536;
}

__global__ void compute_depth_kernel(
        const vec3* positions,
        short* depths,
        int* indices,
        const int N,
        const float* proj
        )
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= N) return;

    depths[idx] = compute_depth(positions[idx], proj);
    indices[idx] = idx;
}

void project_to_z(const Eigen::Matrix4f& mvp,
                  CudaGLBuffer<float>& positions,
                  short* depthPtr,
                  int* indicesPtr,
                  int N)
{
    float* d_proj;
    cudaMalloc(&d_proj, sizeof(float) * 16);
    cudaMemcpy(d_proj, mvp.data(), sizeof(float) * 16, cudaMemcpyHostToDevice);

    auto [posPtr, posSize] = positions.getSSBOAndSize();

    // 2. Lancer le kernel
    int blockSize = 256;
    int gridSize = (N + blockSize - 1) / blockSize;
    compute_depth_kernel<<<gridSize, blockSize>>>((const vec3*)posPtr, (short*)depthPtr,
                                                  (int*)indicesPtr,
                                                  N, (const float*)d_proj);

    cudaDeviceSynchronize();
    cudaFree(d_proj);
}

bool PointCloudRendererBackend::hasCuda()
{
    int deviceCount = 0;
    cudaGetDeviceCount(&deviceCount);
    return deviceCount > 0;
}

void PointCloudRendererBackend::transform_and_sort_cpu(const Eigen::Matrix4f& P,
                                                       const Eigen::MatrixXf& positions,
                                                       std::vector<float>& depths,
                                                       std::vector<int>& indices)
{
    const Eigen::RowVector3f proj_row = P.row(2).head<3>();

    for(auto index : indices)
    {
        Eigen::Vector3f center = (positions.row(index).transpose());
        depths[index] = proj_row.dot(center);
    }


//    for(size_t i=0;i<indices.size(); ++i)
//    {
//        auto idx = i;
//        Eigen::Vector3f center = (positions.row(idx).transpose());
//        indices[idx] = idx;
//        depths[idx] = proj_row.dot(center);
//    }

    //    tbb::parallel_for(
    //        tbb::blocked_range<size_t>(0, indices.size(), 10000),
    //        [&proj_row, &depths, &positions, &indices](const tbb::blocked_range<size_t>& r)
    //        {
    //            for (size_t i = r.begin(); i != r.end(); ++i)
    //            {
    //                Eigen::Vector3f center = (positions.row(i).transpose());
    //                indices[i] = i;
    //                depths[i] = proj_row.dot(center);
    //            }
    //        }
    //    );

    /*tbb::parallel_for_each(indices, [&proj_row, &depths, &positions](int idx){
        Eigen::Vector3f center = (positions.row(idx).transpose());
        depths[idx] = proj_row.dot(center);
    });*/

    //sort_float_int(depths.data(), indices.data(), depths.size());

    tbb::parallel_sort(indices.begin(), indices.end(),
                       [&](int i, int j) {
        return depths[i] < depths[j];
    });
}
