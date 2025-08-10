#pragma once

// Internal includes
#include "math/types.h"
#include "volume/leb_volume.h"
#include "volume/grid_volume.h"

// External includes
#include <vector>

struct TetraData
{
    // 4 Compressed plane equations (faces 0 -> 3)
    uint4 compressedEquations;
    // 4 neighbors
    uint4 neighbors;
    // Per element density
    float density;
};

// Structure that holds everything we need to ray trace
struct LEBVolumeGPU
{
    // Camera and volume data (used for debug vis)
    bool frustumCull = false;
    float4x4 vpMat = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    float3 cameraPosition = { 0.0, 0.0, 0.0 };

    // Volume scale
    float3 scale = { 1.0, 1.0, 1.0 };

    // Per-tetra data
	std::vector<TetraData> tetraData;
    std::vector<float3> centerArray;
    std::vector<float> densityArray;

    // Outside interface data
    std::vector<uint3> rtasIndexArray;
    std::vector<float3> rtasPositionArray;
    std::vector<uint32_t> outsideElements;

    // Debug data
    std::vector<float3> positionArray;
};

namespace leb_volume
{
    // Convert leb volume CPUto leb volume GPU
    uint64_t convert_to_leb_volume_to_gpu(const LEBVolume& lebVolume, const GridVolume& gridVolume, const FittingParameters& fitParam, uint32_t maxDepth, LEBVolumeGPU& lebVolumeGPU);

    // Import a packed mesh from disk
    void import_leb_volume_gpu(const char* path, LEBVolumeGPU& lebVolumeGPU);

    // Export a packed mesh to disk
    void export_leb_volume_gpu(const LEBVolumeGPU& lebVolumeGPU, const char* path);
}