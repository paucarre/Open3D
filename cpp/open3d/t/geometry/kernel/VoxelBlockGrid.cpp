// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018-2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "open3d/t/geometry/kernel/VoxelBlockGrid.h"

#include <vector>

#include "open3d/core/CUDAUtils.h"
#include "open3d/core/ShapeUtil.h"
#include "open3d/core/Tensor.h"
#include "open3d/core/hashmap/HashMap.h"
#include "open3d/utility/Logging.h"

namespace open3d {
namespace t {
namespace geometry {
namespace kernel {
namespace voxel_grid {

void PointCloudTouch(std::shared_ptr<core::HashMap>& hashmap,
                     const core::Tensor& points,
                     core::Tensor& voxel_block_coords,
                     index_t voxel_grid_resolution,
                     float voxel_size,
                     float sdf_trunc) {
    if (hashmap->IsCPU()) {
        PointCloudTouchCPU(hashmap, points, voxel_block_coords,
                           voxel_grid_resolution, voxel_size, sdf_trunc);
    } else if (hashmap->IsCUDA()) {
        CUDA_CALL(PointCloudTouchCUDA, hashmap, points, voxel_block_coords,
                  voxel_grid_resolution, voxel_size, sdf_trunc);
    } else {
        utility::LogError("Unimplemented device");
    }
}
/*
 * This seems to return some sort of coordinate index
 * that is related to the depth
 *
 */
void DepthTouch(std::shared_ptr<core::HashMap>& hashmap,
                const core::Tensor& depth,
                const core::Tensor& intrinsic,
                const core::Tensor& extrinsic,
                core::Tensor& voxel_block_coords,
                index_t voxel_grid_resolution,
                float voxel_size,
                float sdf_trunc,
                float depth_scale,
                float depth_max,
                index_t stride) {
    if (hashmap->IsCPU()) {
        DepthTouchCPU(hashmap, depth, intrinsic, extrinsic, voxel_block_coords,
                      voxel_grid_resolution, voxel_size, sdf_trunc, depth_scale,
                      depth_max, stride);
    } else if (hashmap->IsCUDA()) {
        CUDA_CALL(DepthTouchCUDA, hashmap, depth, intrinsic, extrinsic,
                  voxel_block_coords, voxel_grid_resolution, voxel_size,
                  sdf_trunc, depth_scale, depth_max, stride);
    } else {
        utility::LogError("Unimplemented device");
    }
}

void UnseenFrustumDeepTouch(std::shared_ptr<core::HashMap> &hashmap,
                    const core::Tensor &depth,
                    const core::Tensor &intrinsic,
                    const core::Tensor &extrinsic,
                    core::Tensor &voxel_block_coords,
                    index_t voxel_grid_resolution,
                    float voxel_size,
                    float sdf_trunc,
                    float depth_scale,
                    float depth_max,
                    index_t stride,
                    float depth_std_times)  {
    //if (hashmap->IsCPU()) {
    //    DepthTouchCPU(hashmap, depth, intrinsic, extrinsic, voxel_block_coords,
    //                  voxel_grid_resolution, voxel_size, sdf_trunc, depth_scale,
    //                  depth_max, stride);
    //} else if (hashmap->IsCUDA()) {
    CUDA_CALL(UnseenFrustumDeepTouchCUDA, hashmap, depth, intrinsic, extrinsic,
                voxel_block_coords, voxel_grid_resolution, voxel_size,
                sdf_trunc, depth_scale, depth_max, stride, depth_std_times);
    //} else {
    //    utility::LogError("Unimplemented device");
    //}
}

void GetVoxelCoordinatesAndFlattenedIndices(const core::Tensor& buf_indices,
                                            const core::Tensor& block_keys,
                                            core::Tensor& voxel_coords,
                                            core::Tensor& flattened_indices,
                                            index_t block_resolution,
                                            float voxel_size) {
    if (block_keys.IsCPU()) {
        GetVoxelCoordinatesAndFlattenedIndicesCPU(
                buf_indices, block_keys, voxel_coords, flattened_indices,
                block_resolution, voxel_size);
    } else if (block_keys.IsCUDA()) {
        CUDA_CALL(GetVoxelCoordinatesAndFlattenedIndicesCUDA, buf_indices,
                  block_keys, voxel_coords, flattened_indices, block_resolution,
                  voxel_size);
    } else {
        utility::LogError("Unimplemented device");
    }
}

#define DISPATCH_VALUE_DTYPE_TO_TEMPLATE(WEIGHT_DTYPE, COLOR_DTYPE, ...)    \
    [&] {                                                                   \
        if (WEIGHT_DTYPE == open3d::core::Float32 &&                        \
            COLOR_DTYPE == open3d::core::Float32) {                         \
            using weight_t = float;                                         \
            using color_t = float;                                          \
            return __VA_ARGS__();                                           \
        } else if (WEIGHT_DTYPE == open3d::core::UInt16 &&                  \
                   COLOR_DTYPE == open3d::core::UInt16) {                   \
            using weight_t = uint16_t;                                      \
            using color_t = uint16_t;                                       \
            return __VA_ARGS__();                                           \
        } else if (WEIGHT_DTYPE == open3d::core::Float32 &&                 \
                   COLOR_DTYPE == open3d::core::UInt16) {                   \
            using weight_t = float;                                         \
            using color_t = uint16_t;                                       \
            return __VA_ARGS__();                                           \
        } else {                                                            \
            utility::LogError(                                              \
                    "Unsupported value data type combination. Expected "    \
                    "(float, float) or (uint16, uint16), but received ({} " \
                    "{}).",                                                 \
                    WEIGHT_DTYPE.ToString(), COLOR_DTYPE.ToString());       \
        }                                                                   \
    }()


#define DISPATCH_INPUT_DTYPE_TO_TEMPLATE(DEPTH_DTYPE, COLOR_DTYPE, ...)        \
    [&] {                                                                      \
        if (DEPTH_DTYPE == open3d::core::Float32 &&                            \
            COLOR_DTYPE == open3d::core::Float32) {                            \
            using input_depth_t = float;                                       \
            using input_color_t = float;                                       \
            return __VA_ARGS__();                                              \
        } else if (DEPTH_DTYPE == open3d::core::UInt16 &&                      \
                   COLOR_DTYPE == open3d::core::UInt8) {                       \
            using input_depth_t = uint16_t;                                    \
            using input_color_t = uint8_t;                                     \
            return __VA_ARGS__();                                              \
        } else {                                                               \
            utility::LogError(                                                 \
                    "Unsupported input data type combination. Expected "       \
                    "(float, float) or (uint16, uint8), but received ({} {})", \
                    DEPTH_DTYPE.ToString(), COLOR_DTYPE.ToString());           \
        }                                                                      \
    }()


#define DISPATCH_VALUE_DTYPE_TO_TEMPLATE_PROB(WEIGHT_DTYPE, COLOR_DTYPE, PROBABILITY_TYPE, ...)    \
    [&] {                                                                   \
        if (WEIGHT_DTYPE == open3d::core::Float32 &&                        \
            COLOR_DTYPE == open3d::core::Float32 &&                         \
            PROBABILITY_TYPE == open3d::core::Float32) {                    \
            using weight_t = float;                                         \
            using color_t = float;                                          \
            using probability_t = float;                                    \
            return __VA_ARGS__();                                           \
        } else if (WEIGHT_DTYPE == open3d::core::UInt16 &&                  \
                   COLOR_DTYPE == open3d::core::UInt16 &&                   \
                   PROBABILITY_TYPE == open3d::core::Float32) {             \
            using weight_t = uint16_t;                                      \
            using color_t = uint16_t;                                       \
            using probability_t = float;                                    \
            return __VA_ARGS__();                                           \
        } else if (WEIGHT_DTYPE == open3d::core::Float32 &&                 \
                   COLOR_DTYPE == open3d::core::UInt16 &&                   \
                   PROBABILITY_TYPE == open3d::core::Float32) {             \
            using weight_t = float;                                      \
            using color_t = uint16_t;                                       \
            using probability_t = float;                                    \
            return __VA_ARGS__();                                           \
        } else {                                                            \
            utility::LogError(                                              \
                    "Unsupported value data type combination. Expected "    \
                    "(float, float) or (uint16, uint16), but received ({} " \
                    "{}).",                                                 \
                    WEIGHT_DTYPE.ToString(), COLOR_DTYPE.ToString());       \
        }                                                                   \
    }()

#define DISPATCH_VALUE_DTYPE_TO_TEMPLATE_DOWN_INT(DEPTH_DTYPE, WEIGHT_DTYPE,  ...)   \
    [&] {                                                                            \
        if (WEIGHT_DTYPE == open3d::core::Float32 &&                                 \
            DEPTH_DTYPE == open3d::core::Float32) {                                  \
            using weight_t = float;                                                  \
            using input_depth_t = float;                                             \
            return __VA_ARGS__();                                                    \
        } else if (WEIGHT_DTYPE == open3d::core::UInt16 &&                           \
                   DEPTH_DTYPE == open3d::core::UInt16) {                            \
            using weight_t = uint16_t;                                               \
            using input_depth_t = uint16_t;                                          \
            return __VA_ARGS__();                                                    \
        } else if (WEIGHT_DTYPE == open3d::core::Float32 &&                          \
                   DEPTH_DTYPE == open3d::core::UInt16) {                            \
            using weight_t = float;                                                  \
            using input_depth_t = uint16_t;                                          \
            return __VA_ARGS__();                                                    \
        } else {                                                                     \
            utility::LogError(                                                       \
                    "Unsupported weight value data type combination. Expected "      \
                    "(float, float) or (uint16, uint16), but received ({} "          \
                    "{}).",                                                          \
                    WEIGHT_DTYPE.ToString(), DEPTH_DTYPE.ToString());                \
        }                                                                            \
    }()

#define DISPATCH_VALUE_DTYPE_TO_TEMPLATE_DOWN_INT_DEALLOCATE(WEIGHT_DTYPE,  ...)   \
    [&] {                                                                            \
        if (WEIGHT_DTYPE == open3d::core::Float32) {                                 \
            using weight_t = float;                                                  \
            return __VA_ARGS__();                                                    \
        } else if (WEIGHT_DTYPE == open3d::core::UInt16) {                           \
            using weight_t = uint16_t;                                               \
            return __VA_ARGS__();                                                    \
        } else {                                                                     \
            utility::LogError(                                                       \
                    "Unsupported weight value data type combination. Expected "      \
                    "(float) or (uint16), but received ({}). ",                      \
                    WEIGHT_DTYPE.ToString());                                        \
        }                                                                            \
    }()


#define DISPATCH_INPUT_DTYPE_TO_TEMPLATE_PROB(DEPTH_DTYPE, COLOR_DTYPE, PROBABILITY_DTYPE, ...)        \
    [&] {                                                                      \
        if (DEPTH_DTYPE == open3d::core::Float32 &&                            \
            COLOR_DTYPE == open3d::core::Float32 &&                            \
            PROBABILITY_DTYPE == open3d::core::Float32) {                       \
            using input_depth_t = float;                                       \
            using input_color_t = float;                                       \
            using input_probability_t = float;                                 \
            return __VA_ARGS__();                                              \
        } else if (DEPTH_DTYPE == open3d::core::UInt16 &&                      \
                   COLOR_DTYPE == open3d::core::UInt8 &&                       \
                   PROBABILITY_DTYPE == open3d::core::Float32) {                \
            using input_depth_t = uint16_t;                                    \
            using input_color_t = uint8_t;                                     \
            using input_probability_t = float;                                 \
            return __VA_ARGS__();                                              \
        } else {                                                               \
            utility::LogError(                                                 \
                    "Unsupported input data type combination. Expected "       \
                    "(float, float, float) or (uint16, uint8, float), but received ({} {} {})", \
                    DEPTH_DTYPE.ToString(), COLOR_DTYPE.ToString(), PROBABILITY_DTYPE.ToString());           \
        }                                                                      \
    }()

void Integrate(const core::Tensor& depth,
               const core::Tensor& color,
               const core::Tensor& probabilities,
               const core::Tensor& block_indices,
               const core::Tensor& block_keys,
               TensorMap& block_value_map,
               const core::Tensor& depth_intrinsic,
               const core::Tensor& color_intrinsic,
               const core::Tensor& probabilities_intrinsic,
               const core::Tensor& extrinsic,
               index_t resolution,
               float voxel_size,
               float sdf_trunc,
               float depth_scale,
               float depth_max) {
    using tsdf_t = float;
    core::Dtype block_weight_dtype = core::Dtype::Float32;
    core::Dtype block_color_dtype = core::Dtype::Float32;
    if (block_value_map.Contains("weight")) {
        block_weight_dtype = block_value_map.at("weight").GetDtype();
    }
    if (block_value_map.Contains("color")) {
        block_color_dtype = block_value_map.at("color").GetDtype();
    }

    core::Dtype input_depth_dtype = depth.GetDtype();
    core::Dtype input_color_dtype = (input_depth_dtype == core::Dtype::Float32)
                                            ? core::Dtype::Float32
                                            : core::Dtype::UInt8;
    core::Dtype input_probability_dtype = probabilities.GetDtype();

    if (color.NumElements() > 0) {
        input_color_dtype = color.GetDtype();
    }

    core::Device::DeviceType device_type = depth.GetDevice().GetType();
    if (device_type == core::Device::DeviceType::CPU) {
        DISPATCH_INPUT_DTYPE_TO_TEMPLATE_PROB(
                input_depth_dtype, input_color_dtype, input_probability_dtype, [&] {
                    DISPATCH_VALUE_DTYPE_TO_TEMPLATE_PROB(
                            block_weight_dtype, block_color_dtype, input_probability_dtype, [&] {
                                IntegrateCPU<input_depth_t, input_color_t, input_probability_t,
                                             tsdf_t, weight_t, color_t, probability_t>(
                                        depth, color, probabilities, block_indices, block_keys,
                                        block_value_map, depth_intrinsic,
                                        color_intrinsic, probabilities_intrinsic,
                                        extrinsic, resolution,
                                        voxel_size, sdf_trunc, depth_scale,
                                        depth_max);
                            });
                });
    } else if (device_type == core::Device::DeviceType::CUDA) {
#ifdef BUILD_CUDA_MODULE
        DISPATCH_INPUT_DTYPE_TO_TEMPLATE_PROB(
                input_depth_dtype, input_color_dtype, input_probability_dtype, [&] {
                    DISPATCH_VALUE_DTYPE_TO_TEMPLATE_PROB(
                            block_weight_dtype, block_color_dtype, input_probability_dtype, [&] {
                                IntegrateCUDA<input_depth_t, input_color_t, input_probability_t,
                                             tsdf_t, weight_t, color_t, probability_t>(
                                        depth, color, probabilities, block_indices, block_keys,
                                        block_value_map, depth_intrinsic,
                                        color_intrinsic, probabilities_intrinsic,
                                        extrinsic, resolution,
                                        voxel_size, sdf_trunc, depth_scale,
                                        depth_max);
                            });
                });
#else
        utility::LogError("Not compiled with CUDA, but CUDA device is used.");
#endif
    } else {
        utility::LogError("Unimplemented device");
    }
}

void Integrate(const core::Tensor& depth,
               const core::Tensor& color,
               const core::Tensor& block_indices,
               const core::Tensor& block_keys,
               TensorMap& block_value_map,
               const core::Tensor& depth_intrinsic,
               const core::Tensor& color_intrinsic,
               const core::Tensor& extrinsic,
               index_t resolution,
               float voxel_size,
               float sdf_trunc,
               float depth_scale,
               float depth_max) {
    using tsdf_t = float;
    core::Dtype block_weight_dtype = core::Dtype::Float32;
    core::Dtype block_color_dtype = core::Dtype::Float32;
    if (block_value_map.Contains("weight")) {
        block_weight_dtype = block_value_map.at("weight").GetDtype();
    }
    if (block_value_map.Contains("color")) {
        block_color_dtype = block_value_map.at("color").GetDtype();
    }

    core::Dtype input_depth_dtype = depth.GetDtype();
    core::Dtype input_color_dtype = (input_depth_dtype == core::Dtype::Float32)
                                            ? core::Dtype::Float32
                                            : core::Dtype::UInt8;
    if (color.NumElements() > 0) {
        input_color_dtype = color.GetDtype();
    }

    if (depth.IsCPU()) {
        DISPATCH_INPUT_DTYPE_TO_TEMPLATE(
                input_depth_dtype, input_color_dtype, [&] {
                    DISPATCH_VALUE_DTYPE_TO_TEMPLATE(
                            block_weight_dtype, block_color_dtype, [&] {
                                IntegrateCPU<input_depth_t, input_color_t,
                                             tsdf_t, weight_t, color_t>(
                                        depth, color, block_indices, block_keys,
                                        block_value_map, depth_intrinsic,
                                        color_intrinsic, extrinsic, resolution,
                                        voxel_size, sdf_trunc, depth_scale,
                                        depth_max);
                            });
                });
    } else if (depth.IsCUDA()) {
#ifdef BUILD_CUDA_MODULE
        DISPATCH_INPUT_DTYPE_TO_TEMPLATE(
                input_depth_dtype, input_color_dtype, [&] {
                    DISPATCH_VALUE_DTYPE_TO_TEMPLATE(
                            block_weight_dtype, block_color_dtype, [&] {
                                IntegrateCUDA<input_depth_t, input_color_t,
                                              tsdf_t, weight_t, color_t>(
                                        depth, color, block_indices, block_keys,
                                        block_value_map, depth_intrinsic,
                                        color_intrinsic, extrinsic, resolution,
                                        voxel_size, sdf_trunc, depth_scale,
                                        depth_max);
                            });
                });
#else
        utility::LogError("Not compiled with CUDA, but CUDA device is used.");
#endif
    } else {
        utility::LogError("Unimplemented device");
    }
}


void DownIntegrate(
         const core::Tensor& depth,
         const core::Tensor& block_indices,
         const core::Tensor& block_keys,
         TensorMap& block_value_map,
         const core::Tensor& depth_intrinsic,
         const core::Tensor& extrinsics,
         index_t resolution,
         float voxel_size,
         float sdf_trunc,
         float depth_scale,
         float depth_max,
         float down_integration_multiplier) {
    using tsdf_t = float;
    core::Dtype block_weight_dtype = core::Dtype::Float32;
    if (block_value_map.Contains("weight")) {
        block_weight_dtype = block_value_map.at("weight").GetDtype();
    }
    core::Dtype input_depth_dtype = depth.GetDtype();
    #ifdef BUILD_CUDA_MODULE
        DISPATCH_VALUE_DTYPE_TO_TEMPLATE_DOWN_INT(
                input_depth_dtype, block_weight_dtype, [&] {
                    DownIntegrateCUDA<input_depth_t, tsdf_t, weight_t>(
                            depth,
                            block_indices,
                            block_keys,
                            block_value_map,
                            depth_intrinsic,
                            extrinsics,
                            resolution,
                            voxel_size,
                            sdf_trunc,
                            depth_scale,
                            depth_max,
                            down_integration_multiplier);
                });
    #endif
}


void Deallocate(
         const core::Tensor& block_indices,
         const core::Tensor& block_keys,
         TensorMap& block_value_map,
         index_t resolution,
         float weight_threshold,
         float occupancy,
         core::Tensor &voxel_block_coords) {
    core::Dtype block_weight_dtype = core::Dtype::Float32;
    if (block_value_map.Contains("weight")) {
        block_weight_dtype = block_value_map.at("weight").GetDtype();
    }
    #ifdef BUILD_CUDA_MODULE
        DISPATCH_VALUE_DTYPE_TO_TEMPLATE_DOWN_INT_DEALLOCATE(
                block_weight_dtype, [&] {
                    DeallocateCUDA<weight_t>(
                            block_indices,
                            block_keys,
                            block_value_map,
                            resolution,
                            weight_threshold,
                            occupancy,
                            voxel_block_coords);
                });
    #endif
}

void EstimateRange(const core::Tensor& block_keys,
                   core::Tensor& range_minmax_map,
                   const core::Tensor& intrinsics,
                   const core::Tensor& extrinsics,
                   int h,
                   int w,
                   int down_factor,
                   int64_t block_resolution,
                   float voxel_size,
                   float depth_min,
                   float depth_max) {
    static const core::Device host("CPU:0");
    core::Tensor intrinsics_d = intrinsics.To(host, core::Float64).Contiguous();
    core::Tensor extrinsics_d = extrinsics.To(host, core::Float64).Contiguous();

    if (block_keys.IsCPU()) {
        EstimateRangeCPU(block_keys, range_minmax_map, intrinsics_d,
                         extrinsics_d, h, w, down_factor, block_resolution,
                         voxel_size, depth_min, depth_max);
    } else if (block_keys.IsCUDA()) {
#ifdef BUILD_CUDA_MODULE
        EstimateRangeCUDA(block_keys, range_minmax_map, intrinsics_d,
                          extrinsics_d, h, w, down_factor, block_resolution,
                          voxel_size, depth_min, depth_max);
#else
        utility::LogError("Not compiled with CUDA, but CUDA device is used.");
#endif
    } else {
        utility::LogError("Unimplemented device");
    }
}

void RayCast(std::shared_ptr<core::HashMap>& hashmap,
             const TensorMap& block_value_map,
             const core::Tensor& range_map,
             TensorMap& renderings_map,
             const core::Tensor& intrinsic,
             const core::Tensor& extrinsic,
             index_t h,
             index_t w,
             index_t block_resolution,
             float voxel_size,
             float depth_scale,
             float depth_min,
             float depth_max,
             float weight_threshold,
             float trunc_voxel_multiplier,
             int range_map_down_factor) {
    using tsdf_t = float;
    core::Dtype block_weight_dtype = core::Dtype::Float32;
    core::Dtype block_color_dtype = core::Dtype::Float32;
    if (block_value_map.Contains("weight")) {
        block_weight_dtype = block_value_map.at("weight").GetDtype();
    }
    if (block_value_map.Contains("color")) {
        block_color_dtype = block_value_map.at("color").GetDtype();
    }

    if (hashmap->IsCPU()) {
        DISPATCH_VALUE_DTYPE_TO_TEMPLATE(
                block_weight_dtype, block_color_dtype, [&] {
                    RayCastCPU<tsdf_t, weight_t, color_t>(
                            hashmap, block_value_map, range_map, renderings_map,
                            intrinsic, extrinsic, h, w, block_resolution,
                            voxel_size, depth_scale, depth_min, depth_max,
                            weight_threshold, trunc_voxel_multiplier,
                            range_map_down_factor);
                });

    } else if (hashmap->IsCUDA()) {
#ifdef BUILD_CUDA_MODULE
        DISPATCH_VALUE_DTYPE_TO_TEMPLATE(
                block_weight_dtype, block_color_dtype, [&] {
                    RayCastCUDA<tsdf_t, weight_t, color_t>(
                            hashmap, block_value_map, range_map, renderings_map,
                            intrinsic, extrinsic, h, w, block_resolution,
                            voxel_size, depth_scale, depth_min, depth_max,
                            weight_threshold, trunc_voxel_multiplier,
                            range_map_down_factor);
                });
#else
        utility::LogError("Not compiled with CUDA, but CUDA device is used.");
#endif
    } else {
        utility::LogError("Unimplemented device");
    }
}

void ExtractPointCloud(const core::Tensor& block_indices,
                       const core::Tensor& nb_block_indices,
                       const core::Tensor& nb_block_masks,
                       const core::Tensor& block_keys,
                       const TensorMap& block_value_map,
                       core::Tensor& points,
                       core::Tensor& normals,
                       core::Tensor& colors,
                       index_t block_resolution,
                       float voxel_size,
                       float weight_threshold,
                       int& valid_size) {
    using tsdf_t = float;
    core::Dtype block_weight_dtype = core::Dtype::Float32;
    core::Dtype block_color_dtype = core::Dtype::Float32;
    if (block_value_map.Contains("weight")) {
        block_weight_dtype = block_value_map.at("weight").GetDtype();
    }
    if (block_value_map.Contains("color")) {
        block_color_dtype = block_value_map.at("color").GetDtype();
    }

    if (block_indices.IsCPU()) {
        DISPATCH_VALUE_DTYPE_TO_TEMPLATE(
                block_weight_dtype, block_color_dtype, [&] {
                    ExtractPointCloudCPU<tsdf_t, weight_t, color_t>(
                            block_indices, nb_block_indices, nb_block_masks,
                            block_keys, block_value_map, points, normals,
                            colors, block_resolution, voxel_size,
                            weight_threshold, valid_size);
                });

    } else if (block_indices.IsCUDA()) {
#ifdef BUILD_CUDA_MODULE
        DISPATCH_VALUE_DTYPE_TO_TEMPLATE(
                block_weight_dtype, block_color_dtype, [&] {
                    ExtractPointCloudCUDA<tsdf_t, weight_t, color_t>(
                            block_indices, nb_block_indices, nb_block_masks,
                            block_keys, block_value_map, points, normals,
                            colors, block_resolution, voxel_size,
                            weight_threshold, valid_size);
                });
#else
        utility::LogError("Not compiled with CUDA, but CUDA device is used.");
#endif
    } else {
        utility::LogError("Unimplemented device");
    }
}

void ExtractDetectionPointCloud(
                       const unsigned int class_index,
                       const float minimum_probability,
                       const core::Tensor& block_indices,
                       const core::Tensor& nb_block_indices,
                       const core::Tensor& nb_block_masks,
                       const core::Tensor& block_keys,
                       const TensorMap& block_value_map,
                       core::Tensor& points,
                       core::Tensor& normals,
                       core::Tensor& colors,
                       core::Tensor& probabilities,
                       index_t block_resolution,
                       float voxel_size,
                       float weight_threshold,
                       index_t& valid_size,
                       core::Tensor& objects_class_index, core::Tensor &background_class_index) {
    using tsdf_t = float;
    core::Dtype block_weight_dtype = core::Dtype::Float32;
    core::Dtype block_color_dtype = core::Dtype::Float32;
    if (block_value_map.Contains("weight")) {
        block_weight_dtype = block_value_map.at("weight").GetDtype();
    }
    if (block_value_map.Contains("color")) {
        block_color_dtype = block_value_map.at("color").GetDtype();
    }
    ;

    if (block_indices.IsCPU()) {
        DISPATCH_VALUE_DTYPE_TO_TEMPLATE(
                block_weight_dtype, block_color_dtype, [&] {
                    ExtractDetectionPointCloudCPU<tsdf_t, weight_t, color_t>(
                            class_index, minimum_probability,
                            block_indices, nb_block_indices, nb_block_masks,
                            block_keys, block_value_map, points, normals,
                            colors, probabilities, block_resolution, voxel_size,
                            weight_threshold, valid_size, objects_class_index, background_class_index);
                });

    } else if (block_indices.IsCUDA()) {
#ifdef BUILD_CUDA_MODULE
        DISPATCH_VALUE_DTYPE_TO_TEMPLATE(
                block_weight_dtype, block_color_dtype, [&] {
                    ExtractDetectionPointCloudCUDA<tsdf_t, weight_t, color_t>(
                            class_index, minimum_probability,
                            block_indices, nb_block_indices, nb_block_masks,
                            block_keys, block_value_map, points, normals,
                            colors, probabilities, block_resolution, voxel_size,
                            weight_threshold, valid_size, objects_class_index, background_class_index);
                });
#else
        utility::LogError("Not compiled with CUDA, but CUDA device is used.");
#endif
    } else {
        utility::LogError("Unimplemented device");
    }
}




void ExtractTriangleMesh(const core::Tensor& block_indices,
                         const core::Tensor& inv_block_indices,
                         const core::Tensor& nb_block_indices,
                         const core::Tensor& nb_block_masks,
                         const core::Tensor& block_keys,
                         const TensorMap& block_value_map,
                         core::Tensor& vertices,
                         core::Tensor& triangles,
                         core::Tensor& vertex_normals,
                         core::Tensor& vertex_colors,
                         index_t block_resolution,
                         float voxel_size,
                         float weight_threshold,
                         int& vertex_count) {
    using tsdf_t = float;
    core::Dtype block_weight_dtype = core::Dtype::Float32;
    core::Dtype block_color_dtype = core::Dtype::Float32;
    if (block_value_map.Contains("weight")) {
        block_weight_dtype = block_value_map.at("weight").GetDtype();
    }
    if (block_value_map.Contains("color")) {
        block_color_dtype = block_value_map.at("color").GetDtype();
    }

    if (block_indices.IsCPU()) {
        DISPATCH_VALUE_DTYPE_TO_TEMPLATE(
                block_weight_dtype, block_color_dtype, [&] {
                    ExtractTriangleMeshCPU<tsdf_t, weight_t, color_t>(
                            block_indices, inv_block_indices, nb_block_indices,
                            nb_block_masks, block_keys, block_value_map,
                            vertices, triangles, vertex_normals, vertex_colors,
                            block_resolution, voxel_size, weight_threshold,
                            vertex_count);
                });
    } else if (block_indices.IsCUDA()) {
#ifdef BUILD_CUDA_MODULE
        DISPATCH_VALUE_DTYPE_TO_TEMPLATE(
                block_weight_dtype, block_color_dtype, [&] {
                    ExtractTriangleMeshCUDA<tsdf_t, weight_t, color_t>(
                            block_indices, inv_block_indices, nb_block_indices,
                            nb_block_masks, block_keys, block_value_map,
                            vertices, triangles, vertex_normals, vertex_colors,
                            block_resolution, voxel_size, weight_threshold,
                            vertex_count);
                });
#else
        utility::LogError("Not compiled with CUDA, but CUDA device is used.");
#endif
    } else {
        utility::LogError("Unimplemented device");
    }
}

}  // namespace voxel_grid
}  // namespace kernel
}  // namespace geometry
}  // namespace t
}  // namespace open3d
