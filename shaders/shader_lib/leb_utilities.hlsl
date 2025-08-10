#ifndef LEB_UTILITIES_HLSL
#define LEB_UTILITIES_HLSL

// Includes
#include "shader_lib/intersection.hlsl"

// Useful for debugging
#define DIRECTION_USE_SM

#if defined(LEB_COMPRESSED_NEIGHBORS)
#define INVALID_NEIGHBOR 0xFFFFFF
#define NEIGHBORS_TYPE uint3
#else
#define INVALID_NEIGHBOR 0xFFFFFFFF
#define NEIGHBORS_TYPE uint4
#endif

// Tetra data
struct TetraData
{
    uint4 equations;
    NEIGHBORS_TYPE cmpNeighbors;
    float density;
};

// SRVs
StructuredBuffer<TetraData> _TetraDataBuffer0 : register(TETRA_BUFFER_0_BINDING_SLOT);
StructuredBuffer<TetraData> _TetraDataBuffer1 : register(TETRA_BUFFER_1_BINDING_SLOT);

#if defined(DIRECTION_BUFFER_BINDING_SLOT)
StructuredBuffer<float> _DirectionBuffer: register(DIRECTION_BUFFER_BINDING_SLOT);
#endif

// Ray directions
groupshared float gs_DirectionsRAW[27];

// Array that tells us which vertices define the 4 triangles of the tetrahedron
static const uint3 g_TriangleIndices[4] = {uint3(0, 1, 2), uint3(0, 1, 3), uint3(1, 3, 2), uint3(3, 0, 2)};

NEIGHBORS_TYPE compress_neighbors(uint4 neighbors)
{
#if defined(LEB_COMPRESSED_NEIGHBORS)
    uint3 cmpNeighbors;
    cmpNeighbors.x = neighbors.x & 0xFFFFFF;
    cmpNeighbors.x |= (neighbors.w & 0x000000FF) << 24;

    cmpNeighbors.y = neighbors.y & 0xFFFFFF;
    cmpNeighbors.y |= (neighbors.w & 0x0000FF00) << 16;

    cmpNeighbors.z = neighbors.z & 0xFFFFFF;
    cmpNeighbors.z |= (neighbors.w & 0x00FF0000) << 8;
    return cmpNeighbors;
#else
    return neighbors;
#endif
}

uint4 decompress_neighbors(NEIGHBORS_TYPE compressedNeighbors)
{
#if defined(LEB_COMPRESSED_NEIGHBORS)
    uint4 neighbors;
    neighbors.x = compressedNeighbors.x & 0xFFFFFF;
    neighbors.y = compressedNeighbors.y & 0xFFFFFF;
    neighbors.z = compressedNeighbors.z & 0xFFFFFF;
    neighbors.w = ((compressedNeighbors.x & 0xff000000) >> 24) | ((compressedNeighbors.y & 0xff000000) >> 16) | ((compressedNeighbors.z & 0xff000000) >> 8);
    return neighbors;
#else
    return compressedNeighbors;
#endif
}

uint compress_plane_equation(uint32_t planeIdx, float offsetToOrigin)
{
    // Is it the second set of directions or the first?
    uint signV = planeIdx / 9;
    uint planeID = planeIdx % 9;

    // Compress the plane equation
    return (asuint(offsetToOrigin) & 0xFFFFFFE0) | (signV << 4) | planeID;
}

#if defined(DIRECTION_BUFFER_BINDING_SLOT)
void load_direction_to_sm(uint groupIndex)
{
#if defined(DIRECTION_USE_SM)
    if (groupIndex < 27)
        gs_DirectionsRAW[groupIndex] = _DirectionBuffer[groupIndex];
    GroupMemoryBarrierWithGroupSync();
#endif
}
#endif

#if defined(DIRECTION_BUFFER_BINDING_SLOT)
void decompress_plane_equation(uint cE, out float3 planeDir, out float offsetToOrigin)
{
    uint normIdx = cE & 0xf;

    // Read it from shared memory
    uint dirOffset = 3 * normIdx;
#if defined(DIRECTION_USE_SM)
    planeDir = float3(gs_DirectionsRAW[dirOffset], gs_DirectionsRAW[dirOffset + 1], gs_DirectionsRAW[dirOffset + 2]);
#else
    planeDir = float3(_DirectionBuffer[dirOffset], _DirectionBuffer[dirOffset + 1], _DirectionBuffer[dirOffset + 2]);
#endif
    // Inverse if required
    planeDir *= (cE & 0x10) ? -1.0 : 1.0;

    // offset to origin
    offsetToOrigin = asfloat(cE & 0xFFFFFFE0);
}
#endif

float leb_density(in TetraData data)
{
    return data.density * _DensityMultiplier;
}

TetraData get_tetra_data(uint32_t currentPrimitive)
{
    // Read the data
    uint32_t page = currentPrimitive / 100663296;
    uint32_t cellIndex = currentPrimitive % 100663296;
    if (page == 0)
        return _TetraDataBuffer0[cellIndex];
    else
        return _TetraDataBuffer1[cellIndex];
}

#if defined(DIRECTION_BUFFER_BINDING_SLOT)
float integrate_density(float3 rayOrigin, float3 rayDir, uint32_t currentPrimitive)
{
    // Initialize our loop
    float totalDensity = 0.0;
    uint32_t prevPrimitive = INVALID_NEIGHBOR;

    // March our structure
    float prevL = 0.0;
    while (currentPrimitive != INVALID_NEIGHBOR)
    {
        // Read the additional data
        TetraData data = get_tetra_data(currentPrimitive);
        uint4 neighbors = decompress_neighbors(data.cmpNeighbors);

        // If we hiy the first face
        float l = FLT_MAX;

        // Process the faces
        uint32_t candidate = INVALID_NEIGHBOR;
        [unroll]
        for (uint32_t faceIdx = 0; faceIdx < 4; ++faceIdx)
        {
            if ((neighbors[faceIdx] == INVALID_NEIGHBOR || neighbors[faceIdx] != prevPrimitive))
            {
                // Get the plane equation
                float3 planeDir;
                float offset;
                decompress_plane_equation(data.equations[faceIdx], planeDir, offset);

                // Intersect
                float t = ray_plane_intersection(rayOrigin, rayDir, planeDir, offset);
                if (t < l)
                {
                    l = t;
                    candidate = neighbors[faceIdx];
                }
            }
        }

        // Current step
        float cl = l - prevL;

        // Update the primitive
        prevPrimitive = currentPrimitive;
        currentPrimitive = candidate;

        // Move along the ray
        totalDensity += max(cl, 0) * leb_density(data);
        
        // Move along the ray
        prevL += cl;
    }

    return totalDensity;
}
#endif

#endif // LEB_UTILITIES_HLSL