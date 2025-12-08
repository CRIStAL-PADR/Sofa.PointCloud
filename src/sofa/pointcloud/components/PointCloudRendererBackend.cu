#include <cub/cub.cuh>
#include <thrust/device_vector.h>
#include <thrust/sort.h>

void sort_float_int(float* h_keys, int* h_values, int N)
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

#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <cuda_gl_interop.h>

void sort_float_int_to_ssbo(
    const float* h_keys,
    const int*   h_values,
    int N,
    GLuint ssbo
) {
    // ---------------------------------------------------------------------
    // 1. Copier les données de l’hôte vers le GPU (device_vector)
    // ---------------------------------------------------------------------
    thrust::device_vector<float> d_keys(h_keys, h_keys + N);
    thrust::device_vector<int>   d_vals(h_values, h_values + N); // indices

    // ---------------------------------------------------------------------
    // 2. Tri Thrust : les valeurs suivent l'ordre des clés
    // ---------------------------------------------------------------------
    thrust::sort_by_key(d_keys.begin(), d_keys.end(), d_vals.begin());

    // ---------------------------------------------------------------------
    // 3. Register OpenGL SSBO ↔ CUDA
    // ---------------------------------------------------------------------
    cudaGraphicsResource* cuda_res;
    cudaGraphicsGLRegisterBuffer(
        &cuda_res,
        ssbo,
        cudaGraphicsRegisterFlagsWriteDiscard   // on écrase entièrement le ssbo
    );

    // Map pour accéder en CUDA
    cudaGraphicsMapResources(1, &cuda_res);

    int* ssbo_ptr = nullptr;
    size_t ssbo_size = 0;
    cudaGraphicsResourceGetMappedPointer(
        (void**)&ssbo_ptr,
        &ssbo_size,
        cuda_res
    );

    // Vérification de taille
    if (ssbo_size < N * sizeof(int)) {
        cudaGraphicsUnmapResources(1, &cuda_res);
        cudaGraphicsUnregisterResource(cuda_res);
        throw std::runtime_error("SSBO is too small for sorted indices!");
    }

    // ---------------------------------------------------------------------
    // 4. Copier UNIQUEMENT les indices triés dans le SSBO
    // ---------------------------------------------------------------------
    cudaMemcpy(
        ssbo_ptr,
        thrust::raw_pointer_cast(d_vals.data()), // uniquement les indices
        N * sizeof(int),
        cudaMemcpyDeviceToDevice
    );

    // ---------------------------------------------------------------------
    // 5. Cleanup
    // ---------------------------------------------------------------------
    cudaGraphicsUnmapResources(1, &cuda_res);
    cudaGraphicsUnregisterResource(cuda_res);
}

#include <cuda_runtime.h>

struct vec3 { float x, y, z; };
struct mat4 { float m[16]; }; // row-major

__device__ float compute_depth(const vec3& pos, const mat4& proj)
{
    // Transforme le vertex en vec4
    float x = pos.x;
    float y = pos.y;
    float z = pos.z;
    float w = 1.0f;

    // Multiplication matrice 4x4
    float clip_x = proj.m[0]*x + proj.m[4]*y + proj.m[8]*z + proj.m[12]*w;
    float clip_y = proj.m[1]*x + proj.m[5]*y + proj.m[9]*z + proj.m[13]*w;
    float clip_z = proj.m[2]*x + proj.m[6]*y + proj.m[10]*z + proj.m[14]*w;
    float clip_w = proj.m[3]*x + proj.m[7]*y + proj.m[11]*z + proj.m[15]*w;

    // Depth normalisée (NDC z)
    return clip_z / clip_w;
}

__global__ void compute_depth_kernel(
    const vec3* positions,
    float* depths,
    int N,
    mat4 proj
)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= N) return;

    depths[idx] = compute_depth(positions[idx], proj);
}

void compute_depths_cuda(
    const std::vector<vec3>& positions_cpu,
    const mat4& proj,
    float* d_depths  // tableau GPU déjà alloué
)
{
    int N = positions_cpu.size();

    // 1. Copier positions sur GPU
    vec3* d_positions;
    cudaMalloc(&d_positions, N * sizeof(vec3));
    cudaMemcpy(d_positions, positions_cpu.data(), N * sizeof(vec3), cudaMemcpyHostToDevice);

    // 2. Lancer le kernel
    int blockSize = 256;
    int gridSize = (N + blockSize - 1) / blockSize;
    compute_depth_kernel<<<gridSize, blockSize>>>(d_positions, d_depths, N, proj);

    cudaDeviceSynchronize();

    // 3. Libération mémoire positions GPU
    cudaFree(d_positions);
}
