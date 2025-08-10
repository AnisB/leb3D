#pragma once

// Internal includes
#include "volume/leb_volume.h"
#include "volume/grid_volume.h"
#include "volume/heuristic_cache.h"

namespace leb_volume
{
    // Sample the grid at a single value
    float evaluate_grid_value(const GridVolume& gridVolume, float3 position);

    // Get the means in the tetrahedron
    float mean_density_element(const GridVolume& gridVolume, uint32_t depth, const Tetrahedron& tetra);

    // Fit volume to grid
    uint32_t fit_volume_to_grid(LEBVolume& lebVolume, const GridVolume& gridVolume, const HeuristicCache& heuristicCache, const FittingParameters& parameters);
}