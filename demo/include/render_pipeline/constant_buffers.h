#pragma once

// SDK includes
#include "math/types.h"

struct GlobalCB
{
    // View projection matrix
    float4x4 _ViewProjectionMatrix;

    // Inverse view projection matrix
    float4x4 _InvViewProjectionMatrix;

    // Camera position in double as we are operating on planeraty coordinates
    float3 _CameraPosition;
    uint32_t _FrameIndex;

    // Screen size and inverse screen size
    float4 _ScreenSize;

    // Accumulation Factors
    float2 _FrameAccumulationFactors;
    // Density multiplier
    float _DensityMultiplier;
    // Volume albedo
    float _VolumeAlbedo;

    // Sun intensity
    float _SunIntensity;
    // Sky intensity
    float _SkyIntensity;
    // Padding
    float2 _PaddingGB0;

    // Padding
    float3 _SunDirection;
    float _PaddingGB1;
};

struct UpdateCB
{
    // View projection matrix used for the update
    float4x4 _UpdateViewProjectionMatrix;

    // Inverse View projection matrix used for the update
    float4x4 _UpdateInvViewProjectionMatrix;

    // Camera position used for the update
    float3 _UpdateCameraPosition;
    // FOV
    float _UpdateFOV;

    // Volume min
    float3 _VolumeMinPosition;
    // Far plane
    float _UpdateFarPlaneDistance;

    // Volume max
    float3 _VolumeMaxPosition;
    // Padding
    float _PaddingUB0;
};

struct LEBCB
{
    // Number of tetrahedrons in our structure
    uint32_t _NumTetrahedrons;
    // Initial primitive
    uint32_t _InitialPrimitive;
    float2 _PaddingLB0;

    // Volume scale
    float3 _LEBScale;

    // Padding
    float _PaddingLB1;
};

struct GridCB
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