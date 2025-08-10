#pragma once

// Internal includes
#include "math/types.h"
#include "volume/leb_3d_cache.h"

// External includes
#include <stdint.h>
#include <vector>

// Flags for the subdivision routine
#define ELEMENT_INCLUDED 0x1
#define ELEMENT_INVALID_CACHE 0x2
#define ELEMENT_REQUESTED 0x4

struct Diamond
{
    uint64_t heapID[8];
    uint32_t size;
};

struct Tetrahedron
{
    float3 p[4];
};

struct LEBVolume
{
    // Total number of elements
    uint32_t totalNumElements = 0;

    // Minimal depth of the mesh
    uint32_t minimalDepth = 0;

    // Bisector
    std::vector<uint64_t> heapIDArray;
    std::vector<uint8_t> typeArray;
    std::vector<uint4> neighborsArray;

    // Base attributes
    std::vector<float3> basePoints;
    std::vector<uint8_t> baseTypes;

    // Used for subdivision
    std::vector<Tetrahedron> tetraCacheArray;
    std::vector<uint8_t> modifArray;
    std::vector<uint8_t> depthArray;

    // Debug Attribute to track diamond splits
    std::vector<Diamond> diamonds;
};

struct FittingParameters
{
    // Should we fit using a frustum?
    bool frustumCull = false;

    // Should we cull using pixel size
    bool pixelCull = false;

    // Camera position
    float3 cameraPosition;

    // Camera projection matrix
    float4x4 viewProjectionMatrix;

    // Screen Size
    uint2 screenSize;

    // Data used for the fitting
    float ratioThreshold = 2.0f;
    float minThreshold = 5.0f;
    float pixelSize = 5.0f;
};

namespace leb_volume
{
    // Creates the base leb structure for a cube
    void create_type0_cube(LEBVolume& lebVolume);

    // Function that will, for every element, evaluate the 4 vertices of each tetrahedron
    void evaluate_positions(const LEBVolume& lebVolume, std::vector<float3>& vertices);

    // Function that returns if two types are equivalent
    bool equivalent_types(uint8_t type0, uint8_t type1);

    // Evaluate tetrahedron postion
    void evaluate_tetrahedron(uint64_t heapID, uint32_t minDepth, const std::vector<float3>& basePoints, const std::vector<uint8_t>& baseTypes, Tetrahedron& tetra);
    void evaluate_tetrahedron(uint64_t heapID, uint32_t minDepth, const std::vector<float3>& basePoints, const std::vector<uint8_t>& baseTypes, const Leb3DCache& cache, Tetrahedron& tetra);
}