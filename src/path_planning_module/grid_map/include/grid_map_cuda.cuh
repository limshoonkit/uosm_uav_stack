/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 * Portions adapted from EGO-Planner-v2: https://github.com/ZJU-FAST-Lab/EGO-Planner-v2
 */

#ifndef GRID_MAP_CUDA_CUH_
#define GRID_MAP_CUDA_CUH_

#include <vector_types.h>
#include "cuda_util.cuh"

struct GridMapParamsGpu_t
{
    double3 map_origin;
    double3 map_size;
    int3 map_voxel_num;
    double resolution;
    double resolution_inv;
    double obstacles_inflation;
    double3 local_update_range;
    double ground_height;
    double virtual_ceil_height;
};

struct GridMapDataGpu_t
{
    char *occupancy_buffer_inflate; // Device pointer
    float3 *d_bounds_min_max_ptr;   // [0] is min, [1] is max;
};

// --- Kernel Launchers ---
void launchResetBufferKernel(
    char *d_occupancy_buffer_inflate,
    int3 min_id,
    int3 max_id,
    const GridMapParamsGpu_t &params_gpu);

void launchCloudUpdateKernel(
    char *d_occupancy_buffer_inflate,
    const float3 *d_cloud_points,
    int num_points,
    double3 camera_pos_gpu,
    const GridMapParamsGpu_t &params_gpu,
    float3 *d_bounds_min_gpu, // Pointer to device memory for min bounds
    float3 *d_bounds_max_gpu  // Pointer to device memory for max bounds
);

void launchVirtualCeilingKernel(
    char *d_occupancy_buffer_inflate,
    int3 local_bound_min_idx,
    int3 local_bound_max_idx,
    int ceil_id_z,
    const GridMapParamsGpu_t &params_gpu);

// For Visualization
void launchCountOccupiedKernel(
    const char *d_occupancy_buffer_inflate,
    int3 min_cut_idx,
    int3 max_cut_idx,
    const GridMapParamsGpu_t &params_gpu,
    unsigned int *d_point_count);

void launchExtractPointsKernel(
    const char *d_occupancy_buffer_inflate,
    int3 min_cut_idx,
    int3 max_cut_idx,
    const GridMapParamsGpu_t &params_gpu,
    float3 *d_output_points,
    unsigned int *d_current_idx // For atomic increment
);

// --- Helper functions for memory management ---
void allocateCudaMemory(GridMapDataGpu_t &data_gpu, size_t buffer_size);
void freeCudaMemory(GridMapDataGpu_t &data_gpu);
extern "C" void copyParamsToGpu(GridMapParamsGpu_t &params_gpu_device,
                                const Eigen::Vector3d &map_origin,
                                const Eigen::Vector3i &map_voxel_num,
                                double resolution,
                                double resolution_inv,
                                const Eigen::Vector3d &map_size,
                                double obstacles_inflation,
                                const Eigen::Vector3d &local_update_range,
                                double ground_height,
                                double virtual_ceil_height);

#endif // GRID_MAP_CUDA_CUH_