#ifndef CUDA_UTIL_CUH_
#define CUDA_UTIL_CUH_

#include <cstdio>
#include <cuda_runtime.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#define CHECK_CUDA_ERROR(val) checkCuda((val), #val, __FILE__, __LINE__)
inline void checkCuda(cudaError_t result, char const *const func, const char *const file, int const line)
{
    if (result)
    {
        fprintf(stderr, "CUDA error at %s:%d code=%d(%s) \"%s\" \n",
                file, line, static_cast<unsigned int>(result), cudaGetErrorName(result), func);
        cudaDeviceReset();
        exit(EXIT_FAILURE);
    }
}

// Conversion helpers
static inline int3 eigenToInt3(const Eigen::Vector3i &v) { return make_int3(v.x(), v.y(), v.z()); }
static inline double3 eigenToDouble3(const Eigen::Vector3d &v) { return make_double3(v.x(), v.y(), v.z()); }
static inline float3 eigenToFloat3(const Eigen::Vector3d &v) { return make_float3(static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z())); }
static inline Eigen::Vector3d float3ToEigen(const float3 &v) { return Eigen::Vector3d(v.x, v.y, v.z); }

#endif // CUDA_UTIL_CUH_