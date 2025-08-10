#pragma once

// SDK includes
#include "math/types.h"

struct Camera
{
    // Projection parameters
    float fov;
    float2 nearFar;
    float aspectRatio;

    // View parameters
    float3 position;
    float3 angles;

    // Zoom parameters
    float3 scaleOffset;

    // Transformation matrices
    float4x4 view;
    float4x4 projection;

    // Compound matrices
    float4x4 viewProjection;
    float4x4 invViewProjection;
};
