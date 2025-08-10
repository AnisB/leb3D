#ifndef GRID_UTILITIES_HLSL
#define GRID_UTILITIES_HLSL

// SRVs
#if defined(DENSITY_BUFFER_0_BINDING_SLOT)
StructuredBuffer<float> _DensityBuffer0 : register(DENSITY_BUFFER_0_BINDING_SLOT);
#endif

#if defined(DENSITY_BUFFER_1_BINDING_SLOT)
StructuredBuffer<float> _DensityBuffer1 : register(DENSITY_BUFFER_1_BINDING_SLOT);
#endif

#if defined(GRID_CB_BINDING_SLOT)
cbuffer _GridCB : register(GRID_CB_BINDING_SLOT)
{
    // Resolution of the grid
    uint3 _GridResolution;
    
    // Padding
    float _PaddingGRB0;

    // Grid min position
    float3 _GridMinPosition;
    
    // Padding
    float _PaddingGRB1;

    // Grid max position
    float3 _GridMaxPosition;
    
    // Padding
    float _PaddingGRB2;

    // Grid scale position
    float3 _GridScale;
    
    // Padding
    float _PaddingGRB3;
};

int3 evaluate_cell_coords(float3 position)
{
    // Compute the normalized positon
    float3 normalizedPosition = (position - _GridMinPosition) / (_GridMaxPosition - _GridMinPosition);

    // Convert to the coordinates
    return clamp((int3)(normalizedPosition * _GridResolution), 0, _GridResolution - 1);
}

uint32_t cell_coord_to_index(int3 cellCoord)
{
    uint3 coords = cellCoord;
    return coords.x + coords.y * _GridResolution.x + coords.z * _GridResolution.x * _GridResolution.y;
}

bool valid_cell_coord(int3 coord)
{
    return coord.x >= 0 && coord.x < _GridResolution.x 
    && coord.y >= 0 && coord.y < _GridResolution.y 
    && coord.z >= 0 && coord.z < _GridResolution.z;
}

#if defined(DENSITY_BUFFER_0_BINDING_SLOT) && defined(DENSITY_BUFFER_1_BINDING_SLOT)
float grid_density(int3 cellCoord)
{
    // Evaluate the global index
    uint32_t globalIdx = cell_coord_to_index(cellCoord);

    // Read the data
    uint32_t page = globalIdx / 536870912;
    uint32_t cellIndex = globalIdx & 0x1FFFFFFF;
    if (page == 0)
        return _DensityBuffer0[cellIndex] * _DensityMultiplier;
    else
        return _DensityBuffer1[cellIndex] * _DensityMultiplier;
}
#endif

#if defined(DENSITY_BUFFER_0_BINDING_SLOT) && defined(DENSITY_BUFFER_1_BINDING_SLOT)
float integrate_density(float3 rayOrigin, float3 rayDir, int3 currentCellCoords)
{
    // Initialize our loop
    float totalDensity = 0.0f;
    int3 moveIdx = sign(rayDir);

    // March our structure
    while (valid_cell_coord(currentCellCoords))
    {
        // Evaluate the density of the current cell
        float density = grid_density(currentCellCoords);

        // Compute the normalized grid positon
        float3 positionGS = (rayOrigin - _GridMinPosition) / (_GridMaxPosition - _GridMinPosition) * _GridResolution;

        // Compute the position in the cell
        float3 positionInCell = positionGS - currentCellCoords;

        // Which direction are we moving in?
        if (moveIdx.x >= 0)
            positionInCell.x = 1.0 - positionInCell.x;

        if (moveIdx.y >= 0)
            positionInCell.y = 1.0 - positionInCell.y;

        if (moveIdx.z >= 0)
            positionInCell.z = 1.0 - positionInCell.z;

        // Project this position along each ray axis
        float3 projected = positionInCell / abs(rayDir) / _GridResolution;

        // Take the minimal dimension of the 3
        float t = 0.0;
        if (projected.x <= projected.y && projected.x <= projected.z)
        {
            t = projected.x;
            currentCellCoords.x += moveIdx.x;
        }
        else if (projected.y <= projected.x && projected.y <= projected.z)
        {
            t = projected.y;
            currentCellCoords.y += moveIdx.y;
        }
        else
        {
            t = projected.z;
            currentCellCoords.z += moveIdx.z;
        }

        // Move along the and accumulate the density
        rayOrigin += t * rayDir;
        totalDensity += t * density;
    }

    return totalDensity;
}
#endif
#endif

#endif // GRID_UTILITIES_HLSL