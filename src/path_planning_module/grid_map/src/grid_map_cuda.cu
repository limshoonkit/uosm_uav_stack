#include "../include/grid_map_cuda.cuh"
#include <cuda_runtime.h>
#include <device_launch_parameters.h> // For blockIdx, threadIdx, etc.

__device__ inline int3 posToGlobalIndexDevice(const double3 &pos, const GridMapParamsGpu_t &params)
{
    return make_int3(
        static_cast<int>(floor((pos.x - params.map_origin.x) * params.resolution_inv)),
        static_cast<int>(floor((pos.y - params.map_origin.y) * params.resolution_inv)),
        static_cast<int>(floor((pos.z - params.map_origin.z) * params.resolution_inv)));
}

__device__ inline bool isInMapDevice(const int3 &idx, const GridMapParamsGpu_t &params)
{
    return (idx.x >= 0 && idx.x < params.map_voxel_num.x &&
            idx.y >= 0 && idx.y < params.map_voxel_num.y &&
            idx.z >= 0 && idx.z < params.map_voxel_num.z);
}

__device__ inline int toAddressDevice(const int3 &id, const GridMapParamsGpu_t &params)
{
    return id.x * params.map_voxel_num.y * params.map_voxel_num.z +
           id.y * params.map_voxel_num.z +
           id.z;
}

__global__ void resetBufferKernel(
    char *d_occupancy_buffer_inflate,
    int3 min_id,
    int3 max_id,
    GridMapParamsGpu_t params)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x + min_id.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y + min_id.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z + min_id.z;

    if (x <= max_id.x && y <= max_id.y && z <= max_id.z)
    {
        int flat_idx = toAddressDevice(make_int3(x, y, z), params);
        if (flat_idx >= 0 && flat_idx < (params.map_voxel_num.x * params.map_voxel_num.y * params.map_voxel_num.z))
        {
            d_occupancy_buffer_inflate[flat_idx] = 0;
        }
    }
}

__global__ void cloudUpdateKernel(
    char *d_occupancy_buffer_inflate,
    const float3 *d_cloud_points, // Assuming PointXYZ are converted to float3
    int num_points,
    double3 camera_pos_gpu,
    GridMapParamsGpu_t params,
    float3 *d_bounds_min_gpu, // single element array for atomic update
    float3 *d_bounds_max_gpu  // single element array for atomic update
)
{
    int point_idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (point_idx >= num_points)
        return;

    float3 pt = d_cloud_points[point_idx]; // Original point
    double3 p3d = make_double3(pt.x, pt.y, pt.z);
    double3 devi = make_double3(p3d.x - camera_pos_gpu.x, p3d.y - camera_pos_gpu.y, p3d.z - camera_pos_gpu.z);

    // Check if point is within update range (component-wise)
    if (fabs(devi.x) < params.local_update_range.x &&
        fabs(devi.y) < params.local_update_range.y &&
        fabs(devi.z) < params.local_update_range.z)
    {
        const int inf_step = static_cast<int>(ceil(params.obstacles_inflation * params.resolution_inv));
        constexpr int inf_step_z = 1; // As per original code

        for (int ix = -inf_step; ix <= inf_step; ++ix)
        {
            for (int iy = -inf_step; iy <= inf_step; ++iy)
            {
                for (int iz = -inf_step_z; iz <= inf_step_z; ++iz)
                {
                    double3 p3d_inf = make_double3(
                        p3d.x + ix * params.resolution,
                        p3d.y + iy * params.resolution,
                        p3d.z + iz * params.resolution);

                    // Update bounds atomically
                    atomicMin((int *)&d_bounds_min_gpu->x, __float_as_int((float)p3d_inf.x));
                    atomicMin((int *)&d_bounds_min_gpu->y, __float_as_int((float)p3d_inf.y));
                    atomicMin((int *)&d_bounds_min_gpu->z, __float_as_int((float)p3d_inf.z));

                    atomicMax((int *)&d_bounds_max_gpu->x, __float_as_int((float)p3d_inf.x));
                    atomicMax((int *)&d_bounds_max_gpu->y, __float_as_int((float)p3d_inf.y));
                    atomicMax((int *)&d_bounds_max_gpu->z, __float_as_int((float)p3d_inf.z));

                    int3 inf_pt_idx = posToGlobalIndexDevice(p3d_inf, params);

                    if (isInMapDevice(inf_pt_idx, params))
                    {
                        d_occupancy_buffer_inflate[toAddressDevice(inf_pt_idx, params)] = 1;
                    }
                }
            }
        }
    }
}

__global__ void virtualCeilingKernel(
    char *d_occupancy_buffer_inflate,
    int3 local_bound_min_idx,
    int3 local_bound_max_idx,
    int ceil_id_z,
    GridMapParamsGpu_t params)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x + local_bound_min_idx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y + local_bound_min_idx.y;

    if (x <= local_bound_max_idx.x && y <= local_bound_max_idx.y)
    {
        if (ceil_id_z >= 0 && ceil_id_z < params.map_voxel_num.z)
        {
            int3 voxel_idx = make_int3(x, y, ceil_id_z);
            if (isInMapDevice(voxel_idx, params))
            {
                d_occupancy_buffer_inflate[toAddressDevice(voxel_idx, params)] = 1;
            }
        }
    }
}

__global__ void countOccupiedKernel(
    const char *d_occupancy_buffer_inflate,
    int3 min_cut_idx,
    int3 max_cut_idx,
    GridMapParamsGpu_t params,
    unsigned int *d_point_count)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x + min_cut_idx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y + min_cut_idx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z + min_cut_idx.z;

    if (x <= max_cut_idx.x && y <= max_cut_idx.y && z <= max_cut_idx.z)
    {
        int3 current_idx = make_int3(x, y, z);
        if (d_occupancy_buffer_inflate[toAddressDevice(current_idx, params)] == 1)
        {
            atomicAdd(d_point_count, 1);
        }
    }
}

__global__ void extractPointsKernel(
    const char *d_occupancy_buffer_inflate,
    int3 min_cut_idx,
    int3 max_cut_idx,
    GridMapParamsGpu_t params,
    float3 *d_output_points,    // Buffer to store (x,y,z) of occupied cells
    unsigned int *d_current_idx // Atomic counter for placing points in d_output_points
)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x + min_cut_idx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y + min_cut_idx.y;
    int z = blockIdx.z * blockDim.z + threadIdx.z + min_cut_idx.z;

    if (x <= max_cut_idx.x && y <= max_cut_idx.y && z <= max_cut_idx.z)
    {
        int3 current_voxel_idx = make_int3(x, y, z);
        if (d_occupancy_buffer_inflate[toAddressDevice(current_voxel_idx, params)] == 1)
        {
            float pos_x = (x + 0.5f) * static_cast<float>(params.resolution) + static_cast<float>(params.map_origin.x);
            float pos_y = (y + 0.5f) * static_cast<float>(params.resolution) + static_cast<float>(params.map_origin.y);
            float pos_z = (z + 0.5f) * static_cast<float>(params.resolution) + static_cast<float>(params.map_origin.z);

            unsigned int write_idx = atomicAdd(d_current_idx, 1);
            d_output_points[write_idx] = make_float3(pos_x, pos_y, pos_z);
        }
    }
}

// --- Kernel Launchers ---

void launchResetBufferKernel(
    char *d_occupancy_buffer_inflate,
    int3 min_id,
    int3 max_id,
    const GridMapParamsGpu_t &params_gpu)
{
    // adjust block size
    dim3 threads_per_block(8, 8, 8);
    dim3 num_blocks(
        (max_id.x - min_id.x + 1 + threads_per_block.x - 1) / threads_per_block.x,
        (max_id.y - min_id.y + 1 + threads_per_block.y - 1) / threads_per_block.y,
        (max_id.z - min_id.z + 1 + threads_per_block.z - 1) / threads_per_block.z);
    resetBufferKernel<<<num_blocks, threads_per_block>>>(d_occupancy_buffer_inflate, min_id, max_id, params_gpu);
    CHECK_CUDA_ERROR(cudaGetLastError());
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());
}

void launchCloudUpdateKernel(
    char *d_occupancy_buffer_inflate,
    const float3 *d_cloud_points,
    int num_points,
    double3 camera_pos_gpu,
    const GridMapParamsGpu_t &params_gpu,
    float3 *d_bounds_min_gpu,
    float3 *d_bounds_max_gpu)
{
    dim3 threads_per_block(256, 1, 1);
    dim3 num_blocks((num_points + threads_per_block.x - 1) / threads_per_block.x, 1, 1);

    cloudUpdateKernel<<<num_blocks, threads_per_block>>>(
        d_occupancy_buffer_inflate, d_cloud_points, num_points, camera_pos_gpu, params_gpu,
        d_bounds_min_gpu, d_bounds_max_gpu);
    CHECK_CUDA_ERROR(cudaGetLastError());
}

void launchVirtualCeilingKernel(
    char *d_occupancy_buffer_inflate,
    int3 local_bound_min_idx,
    int3 local_bound_max_idx,
    int ceil_id_z,
    const GridMapParamsGpu_t &params_gpu)
{
    dim3 threads_per_block(16, 16, 1); // 2D kernel for X-Y plane
    dim3 num_blocks(
        (local_bound_max_idx.x - local_bound_min_idx.x + 1 + threads_per_block.x - 1) / threads_per_block.x,
        (local_bound_max_idx.y - local_bound_min_idx.y + 1 + threads_per_block.y - 1) / threads_per_block.y,
        1);
    if (num_blocks.x == 0 || num_blocks.y == 0)
        return; // Avoid launching empty grid

    virtualCeilingKernel<<<num_blocks, threads_per_block>>>(
        d_occupancy_buffer_inflate, local_bound_min_idx, local_bound_max_idx, ceil_id_z, params_gpu);
    CHECK_CUDA_ERROR(cudaGetLastError());
}

void launchCountOccupiedKernel(
    const char *d_occupancy_buffer_inflate,
    int3 min_cut_idx,
    int3 max_cut_idx,
    const GridMapParamsGpu_t &params_gpu,
    unsigned int *d_point_count)
{
    CHECK_CUDA_ERROR(cudaMemset(d_point_count, 0, sizeof(unsigned int))); // Reset counter

    dim3 threads_per_block(8, 8, 8); // Tune this
    dim3 num_blocks(
        (max_cut_idx.x - min_cut_idx.x + 1 + threads_per_block.x - 1) / threads_per_block.x,
        (max_cut_idx.y - min_cut_idx.y + 1 + threads_per_block.y - 1) / threads_per_block.y,
        (max_cut_idx.z - min_cut_idx.z + 1 + threads_per_block.z - 1) / threads_per_block.z);
    if (num_blocks.x == 0 || num_blocks.y == 0 || num_blocks.z == 0)
        return;

    countOccupiedKernel<<<num_blocks, threads_per_block>>>(
        d_occupancy_buffer_inflate, min_cut_idx, max_cut_idx, params_gpu, d_point_count);
    CHECK_CUDA_ERROR(cudaGetLastError());
    CHECK_CUDA_ERROR(cudaDeviceSynchronize()); // Ensure count is complete
}

void launchExtractPointsKernel(
    const char *d_occupancy_buffer_inflate,
    int3 min_cut_idx,
    int3 max_cut_idx,
    const GridMapParamsGpu_t &params_gpu,
    float3 *d_output_points,
    unsigned int *d_current_idx)
{
    CHECK_CUDA_ERROR(cudaMemset(d_current_idx, 0, sizeof(unsigned int))); // Reset index

    dim3 threads_per_block(8, 8, 8); // Tune this
    dim3 num_blocks(
        (max_cut_idx.x - min_cut_idx.x + 1 + threads_per_block.x - 1) / threads_per_block.x,
        (max_cut_idx.y - min_cut_idx.y + 1 + threads_per_block.y - 1) / threads_per_block.y,
        (max_cut_idx.z - min_cut_idx.z + 1 + threads_per_block.z - 1) / threads_per_block.z);
    if (num_blocks.x == 0 || num_blocks.y == 0 || num_blocks.z == 0)
        return;

    extractPointsKernel<<<num_blocks, threads_per_block>>>(
        d_occupancy_buffer_inflate, min_cut_idx, max_cut_idx, params_gpu, d_output_points, d_current_idx);
    CHECK_CUDA_ERROR(cudaGetLastError());
    CHECK_CUDA_ERROR(cudaDeviceSynchronize()); // Ensure points are extracted
}

// --- Helper function implementations ---
void allocateCudaMemory(GridMapDataGpu_t &data_gpu_obj, size_t buffer_size)
{
    CHECK_CUDA_ERROR(cudaMalloc(&data_gpu_obj.occupancy_buffer_inflate, buffer_size * sizeof(char)));
    CHECK_CUDA_ERROR(cudaMalloc(&data_gpu_obj.d_bounds_min_max_ptr, 2 * sizeof(float3)));
}

void freeCudaMemory(GridMapDataGpu_t &data_gpu_obj)
{
    if (data_gpu_obj.occupancy_buffer_inflate)
    {
        CHECK_CUDA_ERROR(cudaFree(data_gpu_obj.occupancy_buffer_inflate));
        data_gpu_obj.occupancy_buffer_inflate = nullptr;
    }
    if (data_gpu_obj.d_bounds_min_max_ptr)
    {
        CHECK_CUDA_ERROR(cudaFree(data_gpu_obj.d_bounds_min_max_ptr));
        data_gpu_obj.d_bounds_min_max_ptr = nullptr;
    }
}

void copyParamsToGpu(GridMapParamsGpu_t &params_gpu_device,
                     const Eigen::Vector3d &map_origin,
                     const Eigen::Vector3i &map_voxel_num,
                     double resolution,
                     double resolution_inv,
                     const Eigen::Vector3d &map_size,
                     double obstacles_inflation,
                     const Eigen::Vector3d &local_update_range,
                     double ground_height,
                     double virtual_ceil_height)
{
    params_gpu_device.map_origin = eigenToDouble3(map_origin);
    params_gpu_device.map_voxel_num = eigenToInt3(map_voxel_num);
    params_gpu_device.resolution = resolution;
    params_gpu_device.resolution_inv = resolution_inv;
    params_gpu_device.map_size = eigenToDouble3(map_size);
    params_gpu_device.obstacles_inflation = obstacles_inflation;
    params_gpu_device.local_update_range = eigenToDouble3(local_update_range);
    params_gpu_device.ground_height = ground_height;
    params_gpu_device.virtual_ceil_height = virtual_ceil_height;
}