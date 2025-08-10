// Internal includes
#include "volume/leb_volume.h"
#include "volume/leb_3d_cache.h"
#include "volume/leb_3d_eval.h"
#include "math/operators.h"
#include "tools/security.h"

// External includes
#include <map>

namespace leb_volume
{
    void evaluate_tetrahedron(uint64_t heapID, uint32_t minDepth, const std::vector<float3>& basePoints, const std::vector<uint8_t>& baseTypes, const Leb3DCache& cache, Tetrahedron& tetra)
    {
        // Get the depth of the element
        uint32_t depth = leb__FindMSB(heapID);

        // Compute the required shift to find the original vertices
        uint64_t subTreeDepth = depth - minDepth;

        // Compute the base heapID
        uint64_t baseHeapID = 1uLL << (minDepth);
        uint32_t primitiveID = uint32_t((heapID >> subTreeDepth) - baseHeapID);

        // Heap ID in the sub triangle
        uint64_t mask = subTreeDepth != 0uL ? 0xFFFFFFFFFFFFFFFFull >> (64ull - subTreeDepth) : 0ull;
        uint64_t baseHeap = (1ull << subTreeDepth);
        uint64_t baseMask = (mask & heapID);
        uint64_t subHeapID = baseMask + baseHeap;

        // Grab the base positions of the element
        float3 p0 = basePoints[4 * primitiveID];
        float3 p1 = basePoints[4 * primitiveID + 1];
        float3 p2 = basePoints[4 * primitiveID + 2];
        float3 p3 = basePoints[4 * primitiveID + 3];
        uint8_t baseType = baseTypes[primitiveID];

        // Generate the triangle positions
        float4 baseAttributes[3] = { {p0.x, p1.x, p2.x, p3.x}, {p0.y, p1.y, p2.y, p3.y}, {p0.z, p1.z, p2.z, p3.z} };

        // Decode
        leb_DecodeNodeAttributeArray(subHeapID, baseType, cache.get_cache(), baseAttributes);

        // Fill the child triangle
        tetra.p[0] = float3({ baseAttributes[0].x, baseAttributes[1].x, baseAttributes[2].x });
        tetra.p[1] = float3({ baseAttributes[0].y, baseAttributes[1].y, baseAttributes[2].y });
        tetra.p[2] = float3({ baseAttributes[0].z, baseAttributes[1].z, baseAttributes[2].z });
        tetra.p[3] = float3({ baseAttributes[0].w, baseAttributes[1].w, baseAttributes[2].w });
    }

    void evaluate_tetrahedron(uint64_t heapID, uint32_t minDepth, const std::vector<float3>& basePoints, const std::vector<uint8_t>& baseTypes, Tetrahedron& tetra)
    {
        // Get the depth of the element
        uint32_t depth = leb__FindMSB(heapID);

        // Compute the required shift to find the original vertices
        uint64_t subTreeDepth = depth - minDepth;

        // Compute the base heapID
        uint64_t baseHeapID = 1uLL << (minDepth);
        uint32_t primitiveID = uint32_t((heapID >> subTreeDepth) - baseHeapID);

        // Heap ID in the sub triangle
        uint64_t mask = subTreeDepth != 0uL ? 0xFFFFFFFFFFFFFFFFull >> (64ull - subTreeDepth) : 0ull;
        uint64_t baseHeap = (1ull << subTreeDepth);
        uint64_t baseMask = (mask & heapID);
        uint64_t subHeapID = baseMask + baseHeap;

        // Grab the base positions of the element
        float3 p0 = basePoints[4 * primitiveID];
        float3 p1 = basePoints[4 * primitiveID + 1];
        float3 p2 = basePoints[4 * primitiveID + 2];
        float3 p3 = basePoints[4 * primitiveID + 3];
        uint8_t baseType = baseTypes[primitiveID];

        // Generate the triangle positions
        float4 baseAttributes[3] = { {p0.x, p1.x, p2.x, p3.x}, {p0.y, p1.y, p2.y, p3.y}, {p0.z, p1.z, p2.z, p3.z} };

        // Decode
        leb_DecodeNodeAttributeArray(subHeapID, baseType, baseAttributes);

        // Fill the child triangle
        tetra.p[0] = float3({ baseAttributes[0].x, baseAttributes[1].x, baseAttributes[2].x });
        tetra.p[1] = float3({ baseAttributes[0].y, baseAttributes[1].y, baseAttributes[2].y });
        tetra.p[2] = float3({ baseAttributes[0].z, baseAttributes[1].z, baseAttributes[2].z });
        tetra.p[3] = float3({ baseAttributes[0].w, baseAttributes[1].w, baseAttributes[2].w });
    }

    void evaluate_face_centers(const LEBVolume& lebVolume, std::vector<float3>& faceCenters)
    {
        // Now we are going to check that the faces actually touch each other
        faceCenters.resize(lebVolume.totalNumElements * 4);

        // Now we are going to check that the faces actually touch each other
        for (uint32_t elementID = 0; elementID < lebVolume.totalNumElements; ++elementID)
        {
            // Read the heapID
            uint64_t heapID = lebVolume.heapIDArray[elementID];
            if (heapID == 0)
                continue;

            // Evaluate the positions
            Tetrahedron tetra;
            evaluate_tetrahedron(heapID, lebVolume.minimalDepth, lebVolume.basePoints, lebVolume.baseTypes, tetra);

            // Compute the center of the faces
            faceCenters[4 * elementID] = (tetra.p[0] + tetra.p[1] + tetra.p[2]) / 3.0f;
            faceCenters[4 * elementID + 1] = (tetra.p[0] + tetra.p[3] + tetra.p[1]) / 3.0f;
            faceCenters[4 * elementID + 2] = (tetra.p[1] + tetra.p[3] + tetra.p[2]) / 3.0f;
            faceCenters[4 * elementID + 3] = (tetra.p[0] + tetra.p[2] + tetra.p[3]) / 3.0f;
        }
    }

    void evaluate_neighbors(const LEBVolume& volume, const std::vector<float3>& faceCenters, std::vector<uint4>& neighborsArray)
    {
        // Now we are going to check that the faces actually touch each other
        neighborsArray.resize(volume.totalNumElements);
        memset(neighborsArray.data(), 0xff, volume.totalNumElements * sizeof(uint4));
        for (uint32_t currentID = 0; currentID < volume.totalNumElements; ++currentID)
        {
            // If unallocated, skip
            if (volume.heapIDArray[currentID] == 0)
                continue;

            // Assign the neighbors
            uint4 neighbors = { UINT32_MAX, UINT32_MAX , UINT32_MAX , UINT32_MAX };

            // Compute the center of the faces
            for (uint32_t faceIdx = 0; faceIdx < 4; ++faceIdx)
            {
                float dist = FLT_MAX;
                const float3& faceCenter = faceCenters[4 * currentID + faceIdx];
                for (uint32_t nID = 0; nID < volume.totalNumElements; ++nID)
                {
                    if (nID == currentID)
                        continue;
                    for (uint32_t nfaceIdx = 0; nfaceIdx < 4; ++nfaceIdx)
                    {
                        float cDist = length(faceCenter - faceCenters[4 * nID + nfaceIdx]);
                        if (cDist < 0.001 && cDist < dist)
                        {
                            dist = cDist;
                            at(neighbors, faceIdx) = nID;
                        }
                    }
                }
            }

            neighborsArray[currentID] = neighbors;
        }
    }

    void evaluate_positions(const LEBVolume& lebVolume, std::vector<float3>& vertices)
    {
        Leb3DCache lebCache;

        // Allocate the memory space
        vertices.resize(4 * lebVolume.totalNumElements);
        #pragma omp parallel for num_threads(16)
        for (int32_t elementID = 0; elementID < (int32_t)lebVolume.totalNumElements; ++elementID)
        {
            // Read the heapID
            uint64_t heapID = lebVolume.heapIDArray[elementID];

            // Execute
            Tetrahedron tetra;
            if ((lebVolume.modifArray[elementID] & 0x2) == 2)
                evaluate_tetrahedron(heapID, lebVolume.minimalDepth, lebVolume.basePoints, lebVolume.baseTypes, lebCache, tetra);
            else
                tetra = lebVolume.tetraCacheArray[elementID];

            // Export it to the buffer
            vertices[4 * elementID] = tetra.p[0];
            vertices[4 * elementID + 1] = tetra.p[1];
            vertices[4 * elementID + 2] = tetra.p[2];
            vertices[4 * elementID + 3] = tetra.p[3];
        }
    }

    bool equivalent_types(uint8_t type0, uint8_t type1)
    {
        return (type0 == type1) || (type0 == 1 && type1 == 2) || (type0 == 2 && type1 == 1);
    }

    // Number of tetrahedrons in the base cube structure
    const uint32_t g_BaseTetraCount = 24;
    const uint32_t g_BaseVertexCount = g_BaseTetraCount * 4;

    // Vertex buffer of the base cube tetrahdron
    const float3 g_CubeType0Vertices[g_BaseVertexCount] = { float3({0.000000, 0.000000, 0.000000}), float3({0.000000, 0.000000, -0.500000}),
                                            float3({0.500000, -0.500000, -0.500000}), float3({0.500000, 0.500000, -0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.000000, 0.000000, -0.500000}),
                                            float3({0.500000, 0.500000, -0.500000}), float3({-0.500000, 0.500000, -0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.000000, 0.000000, -0.500000}),
                                            float3({-0.500000, 0.500000, -0.500000}), float3({-0.500000, -0.500000, -0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.000000, 0.000000, -0.500000}),
                                            float3({-0.500000, -0.500000, -0.500000}), float3({0.500000, -0.500000, -0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.500000, 0.000000, 0.000000}),
                                            float3({0.500000, -0.500000, 0.500000}), float3({0.500000, 0.500000, 0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.500000, 0.000000, 0.000000}),
                                            float3({0.500000, 0.500000, 0.500000}), float3({0.500000, 0.500000, -0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.500000, 0.000000, 0.000000}),
                                            float3({0.500000, 0.500000, -0.500000}), float3({0.500000, -0.500000, -0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.500000, 0.000000, 0.000000}),
                                            float3({0.500000, -0.500000, -0.500000}), float3({0.500000, -0.500000, 0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({-0.000000, 0.000000, 0.500000}),
                                            float3({-0.500000, -0.500000, 0.500000}), float3({-0.500000, 0.500000, 0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({-0.000000, 0.000000, 0.500000}),
                                            float3({-0.500000, 0.500000, 0.500000}), float3({0.500000, 0.500000, 0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({-0.000000, 0.000000, 0.500000}),
                                            float3({0.500000, 0.500000, 0.500000}), float3({0.500000, -0.500000, 0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({-0.000000, 0.000000, 0.500000}),
                                            float3({0.500000, -0.500000, 0.500000}), float3({-0.500000, -0.500000, 0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({-0.500000, 0.000000, -0.000000}),
                                            float3({-0.500000, -0.500000, -0.500000}), float3({-0.500000, 0.500000, -0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({-0.500000, 0.000000, -0.000000}),
                                            float3({-0.500000, 0.500000, -0.500000}), float3({-0.500000, 0.500000, 0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({-0.500000, 0.000000, -0.000000}),
                                            float3({-0.500000, 0.500000, 0.500000}), float3({-0.500000, -0.500000, 0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({-0.500000, 0.000000, -0.000000}),
                                            float3({-0.500000, -0.500000, 0.500000}), float3({-0.500000, -0.500000, -0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.000000, 0.500000, 0.000000}),
                                            float3({0.500000, 0.500000, -0.500000}), float3({0.500000, 0.500000, 0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.000000, 0.500000, 0.000000}),
                                            float3({0.500000, 0.500000, 0.500000}), float3({-0.500000, 0.500000, 0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.000000, 0.500000, 0.000000}),
                                            float3({-0.500000, 0.500000, 0.500000}), float3({-0.500000, 0.500000, -0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.000000, 0.500000, 0.000000}),
                                            float3({-0.500000, 0.500000, -0.500000}), float3({0.500000, 0.500000, -0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.000000, -0.500000, 0.000000}),
                                            float3({0.500000, -0.500000, 0.500000}), float3({0.500000, -0.500000, -0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.000000, -0.500000, 0.000000}),
                                            float3({0.500000, -0.500000, -0.500000}), float3({-0.500000, -0.500000, -0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.000000, -0.500000, 0.000000}),
                                            float3({-0.500000, -0.500000, -0.500000}), float3({-0.500000, -0.500000, 0.500000}),
                                            float3({0.000000, 0.000000, 0.000000}), float3({0.000000, -0.500000, 0.000000}),
                                            float3({-0.500000, -0.500000, 0.500000}), float3({0.500000, -0.500000, 0.500000}) };

    void create_type0_cube(LEBVolume& lebVolume)
    {
        // Total number of elements
        lebVolume.totalNumElements = g_BaseTetraCount;

        // Minimal depth of the mesh
        lebVolume.minimalDepth = 5;

        // Allocate memory space
        lebVolume.heapIDArray.resize(g_BaseTetraCount);
        lebVolume.typeArray.resize(g_BaseTetraCount);
        lebVolume.neighborsArray.resize(g_BaseTetraCount);

        // Heap IDs
        for (uint32_t idx = 0; idx < g_BaseTetraCount; ++idx)
            lebVolume.heapIDArray[idx] = 32 + idx;

        // Types
        for (uint32_t idx = 0; idx < g_BaseTetraCount; ++idx)
            lebVolume.typeArray[idx] = 0;

        // Base points
        lebVolume.basePoints.resize(g_BaseVertexCount);
        memcpy(lebVolume.basePoints.data(), g_CubeType0Vertices, sizeof(float3) * g_BaseVertexCount);

        // Base types
        lebVolume.baseTypes.resize(g_BaseTetraCount);
        for (uint32_t idx = 0; idx < g_BaseTetraCount; ++idx)
            lebVolume.baseTypes[idx] = 0;

        // Evaluate the face centers of each face
        std::vector<float3> faceCenters;
        evaluate_face_centers(lebVolume, faceCenters);

        // Now we are going to check that the faces actually touch each other
        lebVolume.neighborsArray.resize(g_BaseTetraCount);
        for (uint32_t currentID = 0; currentID < g_BaseTetraCount; ++currentID)
        {
            if (lebVolume.heapIDArray[currentID] == 0)
            {
                lebVolume.neighborsArray[currentID] = { UINT32_MAX, UINT32_MAX , UINT32_MAX , UINT32_MAX };
                continue;
            }
            uint4 neighbors = { UINT32_MAX, UINT32_MAX , UINT32_MAX , UINT32_MAX };

            // Compute the center of the faces
            for (uint32_t faceIdx = 0; faceIdx < 4; ++faceIdx)
            {
                float dist = FLT_MAX;
                const float3& faceCenter = faceCenters[4 * currentID + faceIdx];
                for (uint32_t nID = 0; nID < g_BaseTetraCount; ++nID)
                {
                    if (nID == currentID)
                        continue;
                    for (uint32_t nfaceIdx = 0; nfaceIdx < 4; ++nfaceIdx)
                    {
                        float cDist = length(faceCenter - faceCenters[4 * nID + nfaceIdx]);
                        if (cDist < 0.001 && cDist < dist)
                        {
                            dist = cDist;
                            at(neighbors, faceIdx) = nID;
                        }
                    }
                }
            }

            // Record the neighbors pointer
            lebVolume.neighborsArray[currentID] = neighbors;
        }

        // Cache structures used for the subdivision
        lebVolume.modifArray.resize(lebVolume.totalNumElements);
        lebVolume.depthArray.resize(lebVolume.totalNumElements);
        lebVolume.tetraCacheArray.resize(lebVolume.totalNumElements);
    }

    bool is_on_external_face(const float3& pt)
    {
        return (abs(pt.x + 0.5f) < 0.00001)
            || (abs(pt.x - 0.5f) < 0.00001)
            || (abs(pt.y + 0.5f) < 0.00001)
            || (abs(pt.y - 0.5f) < 0.00001)
            || (abs(pt.z + 0.5f) < 0.00001)
            || (abs(pt.z - 0.5f) < 0.00001);
    }

    void validate_cubic_volume(const LEBVolume& cpuVolume)
    {
        // Evaluate the face centers of each face
        std::vector<float3> faceCenters;
        evaluate_face_centers(cpuVolume, faceCenters);

        // Build the right neighbor buffer
        std::vector<uint4> neighborsArray;
        evaluate_neighbors(cpuVolume, faceCenters, neighborsArray);

        // Everyone must have a neighbor (except when the face is on an external face of the cube)
        for (uint32_t eleID = 0; eleID < cpuVolume.totalNumElements; ++eleID)
        {
            // Skip for an unallocated element
            if (cpuVolume.heapIDArray[eleID] == 0)
                continue;

            // Grab the geometric neighbors of this element
            uint4 actualNeighbors = neighborsArray[eleID];

            // If the face is  not an external face, it has to have a neighbor
            if (!is_on_external_face(faceCenters[eleID * 4 + 0]))
                assert(actualNeighbors.x != UINT32_MAX);

            // If the face is  not an external face, it has to have a neighbor
            if (!is_on_external_face(faceCenters[eleID * 4 + 1]))
                assert(actualNeighbors.y != UINT32_MAX);

            // If the face is  not an external face, it has to have a neighbor
            if (!is_on_external_face(faceCenters[eleID * 4 + 2]))
                assert(actualNeighbors.z != UINT32_MAX);

            // If the face is  not an external face, it has to have a neighbor
            if (!is_on_external_face(faceCenters[eleID * 4 + 3]))
                assert(actualNeighbors.w != UINT32_MAX);
        }

        // Compare the actual neighbors to the ones we are suppose to have
        for (uint32_t eleID = 0; eleID < cpuVolume.totalNumElements; ++eleID)
        {
            if (cpuVolume.heapIDArray[eleID] == 0)
                continue;

            uint4 actualNeighbors = cpuVolume.neighborsArray[eleID];
            uint4 practialNeighobors = neighborsArray[eleID];

            if ((practialNeighobors.x != actualNeighbors.x)
                || (practialNeighobors.y != actualNeighbors.y)
                || (practialNeighobors.z != actualNeighbors.z)
                || (practialNeighobors.w != actualNeighbors.w))
            {
                /*
                // Put on the stack as much info as possible
                uint8_t cType = cpuVolume.typeArray[eleID];
                uint4 neigh = cpuVolume.neighborsArray[eleID];
                uint8_t n0Type = cpuVolume.typeArray[neigh.x];
                uint8_t n1Type = cpuVolume.typeArray[neigh.y];
                uint8_t n2Type = cpuVolume.typeArray[neigh.z];
                uint8_t n3Type = cpuVolume.typeArray[neigh.w];
                float3 f0 = faceCenters[eleID * 4 + 0];
                float3 f1 = faceCenters[eleID * 4 + 1];
                float3 f2 = faceCenters[eleID * 4 + 2];
                float3 f3 = faceCenters[eleID * 4 + 3];
                */
                assert_fail_msg("Validation failure.");
            }
        }
    }
}
