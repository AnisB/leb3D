// Internal includes
#include "math/operators.h"
#include "volume/leb_volume_gpu.h"
#include "volume/volume_generation.h"
#include "tools/stream.h"

// Mapping of the indices to the faces of the tetrahedrons, ORDER MATTERS HERE
const uint3 g_TriangleIndices[4] = { uint3(0, 1, 2), uint3(3, 1, 0), uint3(1, 3, 2), uint3(3, 0, 2) };

// Directions of the 18 possible orientations of the tetrahedrons planes
const float3 g_Directions[18] = { {-1, 0, 0},
                                        {-0.707107, -0.707107, 0},
                                        {-0.707107, -0, -0.707107},
                                        {-0.707107, 0, 0.707107},
                                        {-0.707107, 0.707107, 0},
                                        {0, -1, 0},
                                        {0, -0.707107, -0.707107},
                                        {0, -0.707107, 0.707107},
                                        {0, 0, -1},

                                        {1, 0, 0},
                                        {0.707107, 0.707107, -0},
                                        {0.707107, 0, 0.707107},
                                        {0.707107, 0, -0.707107},
                                        {0.707107, -0.707107, 0},
                                        {0, 1, 0},
                                        {-0, 0.707107, 0.707107},
                                        {0, 0.707107, -0.707107},
                                        {0, 0, 1},
};

namespace leb_volume
{
    uint32_t compress_plane_equation(uint32_t planeIdx, float offsetToOrigin)
    {
        // Is it the second set of directions or the first?
        uint32_t signV = planeIdx / 9;
        uint32_t planeID = planeIdx % 9;

        // Compress the plane equation
        return (*reinterpret_cast<unsigned int*>(&offsetToOrigin) & 0xFFFFFFE0) | (signV << 4) | planeID;
    }

    float4 evaluate_plane_equation(float3 p1, float3 p2, float3 p3)
    {
        // Calculate two vectors from the three points
        float v1x = p2.x - p1.x, v1y = p2.y - p1.y, v1z = p2.z - p1.z;
        float v2x = p3.x - p1.x, v2y = p3.y - p1.y, v2z = p3.z - p1.z;

        // Compute the cross product of the vectors (v1 × v2)
        float a = v1y * v2z - v1z * v2y;
        float b = v1z * v2x - v1x * v2z;
        float c = v1x * v2y - v1y * v2x;
        float3 normal = normalize(float3(a, b, c));

        // Compute the constant term d = -(a * x1 + b * y1 + c * z1)
        return float4(normal.x, normal.y, normal.z, -dot(normal, p1));
    }

    uint64_t convert_to_leb_volume_to_gpu(const LEBVolume& lebVolume, const GridVolume& gridVolume, const FittingParameters& fitParam, uint32_t maxDepth, LEBVolumeGPU& lebVolumeGPU)
    {
        lebVolumeGPU.frustumCull = fitParam.frustumCull;
        lebVolumeGPU.cameraPosition = fitParam.cameraPosition;
        lebVolumeGPU.vpMat = fitParam.viewProjectionMatrix;
        lebVolumeGPU.scale = gridVolume.scale;

        // Flatten into positions and IDs
        std::vector<float3> positionArray;
        leb_volume::evaluate_positions(lebVolume, positionArray);

        // Allocate the memory space for the attributes
        lebVolumeGPU.positionArray = positionArray;
        lebVolumeGPU.tetraData.resize(lebVolume.totalNumElements);
        lebVolumeGPU.centerArray.resize(lebVolume.totalNumElements);
        lebVolumeGPU.densityArray.resize(lebVolume.totalNumElements);

        // Tracking outside faces
        uint32_t outsideFaceIndex = 0;

        // Process each element
        #pragma omp parallel for num_threads(32)
        for (int32_t eleID = 0; eleID < (int32_t)lebVolume.totalNumElements; ++eleID)
        {
            // Tetra positions
            Tetrahedron tetra;
            tetra.p[0] = positionArray[4 * eleID];
            tetra.p[1] = positionArray[4 * eleID + 1];
            tetra.p[2] = positionArray[4 * eleID + 2];
            tetra.p[3] = positionArray[4 * eleID + 3];
            const float3& center = (tetra.p[0] + tetra.p[1] + tetra.p[2] + tetra.p[3]) * 0.25;
            const uint32_t depth = find_msb_64(lebVolume.heapIDArray[eleID]);

            // Fill the tetra data
            TetraData& data = lebVolumeGPU.tetraData[eleID];
            data.neighbors = lebVolume.neighborsArray[eleID];
            data.density = depth < maxDepth ? leb_volume::mean_density_element(gridVolume, depth, tetra) : leb_volume::evaluate_grid_value(gridVolume, center);
            lebVolumeGPU.centerArray[eleID] = center;

            // Compute and export the plane equations
            for (uint32_t idx = 0; idx < 4; ++idx)
            {
                // Evaluate the equation
                uint3 indices = g_TriangleIndices[idx];
                float4 equation = evaluate_plane_equation(tetra.p[indices.x], tetra.p[indices.y], tetra.p[indices.z]);

                // Classify the direction
                uint32_t tarPlane = 0;
                for (uint32_t plIdx = 0; plIdx < 18; ++plIdx)
                {
                    float candDot = dot(g_Directions[plIdx], xyz(equation));
                    if (candDot > 0.99)
                    {
                        tarPlane = plIdx;
                        break;
                    }
                }

                // Output the plane equation
                at(data.compressedEquations, idx) = compress_plane_equation(tarPlane, equation.w);
            }
        }

        // Process the outside interface in a non parallel fashion
        for (uint32_t eleID = 0; eleID < lebVolume.totalNumElements; ++eleID)
        {
            // Grab the neighbors
            const uint4& neighbors = lebVolumeGPU.tetraData[eleID].neighbors;

            // Log the outside faces
            for (uint32_t faceId = 0; faceId < 4; ++faceId)
            {
                if (at(neighbors, faceId) == UINT32_MAX)
                {
                    // Get the face indices
                    const uint3& indices = g_TriangleIndices[faceId];

                    // Push the outside face data
                    lebVolumeGPU.outsideElements.push_back(eleID);
                    lebVolumeGPU.rtasIndexArray.push_back({ 3 * outsideFaceIndex, 3 * outsideFaceIndex + 1, 3 * outsideFaceIndex + 2});
                    lebVolumeGPU.rtasPositionArray.push_back(positionArray[4 * eleID + indices.x]);
                    lebVolumeGPU.rtasPositionArray.push_back(positionArray[4 * eleID + indices.y]);
                    lebVolumeGPU.rtasPositionArray.push_back(positionArray[4 * eleID + indices.z]);
                    outsideFaceIndex++;
                }
            }
        }

        // Evaluate the compressed size
        uint64_t compressedSize = 0;
        // HeapID
        compressedSize += lebVolume.totalNumElements * sizeof(uint64_t);
        // Neighbors
        compressedSize += lebVolume.totalNumElements * sizeof(uint4);
        // Density
        compressedSize += lebVolumeGPU.densityArray.size() * sizeof(uint32_t);
        return compressedSize;
    }

    void import_leb_volume_gpu(const char* path, LEBVolumeGPU& lebVolume)
    {
        // Vector that will hold our packed mesh 
        std::vector<char> binaryFile;

        // Read from disk
        FILE* pFile;
        pFile = fopen(path, "rb");
        fseek(pFile, 0L, SEEK_END);
        size_t fileSize = _ftelli64(pFile);
        binaryFile.resize(fileSize);
        _fseeki64(pFile, 0L, SEEK_SET);
        rewind(pFile);
        fread(binaryFile.data(), sizeof(char), fileSize, pFile);
        fclose(pFile);

        // Pack the structure in a buffer
        const char* binaryPtr = binaryFile.data();
        unpack_bytes(binaryPtr, lebVolume.frustumCull);
        unpack_bytes(binaryPtr, lebVolume.cameraPosition);
        unpack_bytes(binaryPtr, lebVolume.vpMat);
        unpack_bytes(binaryPtr, lebVolume.scale);

        // Percell data
        unpack_vector_bytes(binaryPtr, lebVolume.tetraData);
        unpack_vector_bytes(binaryPtr, lebVolume.centerArray);
        unpack_vector_bytes(binaryPtr, lebVolume.densityArray);

        // Outside interface data
        unpack_vector_bytes(binaryPtr, lebVolume.rtasIndexArray);
        unpack_vector_bytes(binaryPtr, lebVolume.rtasPositionArray);
        unpack_vector_bytes(binaryPtr, lebVolume.outsideElements);

        // Debug data
        unpack_vector_bytes(binaryPtr, lebVolume.positionArray);
    }

    void export_leb_volume_gpu(const LEBVolumeGPU& lebVolume, const char* path)
    {
        // Vector that will hold our packed mesh 
        std::vector<char> binaryFile;

        // Pack the structure in a buffer
        pack_bytes(binaryFile, lebVolume.frustumCull);
        pack_bytes(binaryFile, lebVolume.cameraPosition);
        pack_bytes(binaryFile, lebVolume.vpMat);
        pack_bytes(binaryFile, lebVolume.scale);

        // Per-tetra data
        pack_vector_bytes(binaryFile, lebVolume.tetraData);
        pack_vector_bytes(binaryFile, lebVolume.centerArray);
        pack_vector_bytes(binaryFile, lebVolume.densityArray);

        // Outside interface data
        pack_vector_bytes(binaryFile, lebVolume.rtasIndexArray);
        pack_vector_bytes(binaryFile, lebVolume.rtasPositionArray);
        pack_vector_bytes(binaryFile, lebVolume.outsideElements);

        // Debug data
        pack_vector_bytes(binaryFile, lebVolume.positionArray);

        // Write to disk
        FILE* pFile;
        pFile = fopen(path, "wb");
        fwrite(binaryFile.data(), sizeof(char), binaryFile.size(), pFile);
        fclose(pFile);
    }
}