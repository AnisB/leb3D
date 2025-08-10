#pragma once

// Internal includes
#include "math/types.h"

// External includes
#include <vector>

struct GridVolume
{
    // Properties of the grid
    float3 scale;
    uint3 resolution;

    // Density array of the cells
    std::vector<float> densityArray;
};

namespace grid_volume
{
    // Export a packed mesh to disk
    void export_grid_volume(const GridVolume& gridVolume, const char* path);

    // Import a packed mesh from disk
    void import_grid_volume(const char* path, GridVolume& gridVolume);
}