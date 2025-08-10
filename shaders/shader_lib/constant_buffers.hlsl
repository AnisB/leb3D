#ifndef CONSTANT_BUFFERS_HLSL
#define CONSTANT_BUFFERS_HLSL

// Global constant buffer
#if defined(GLOBAL_CB_BINDING_SLOT)
cbuffer _GlobalCB : register(GLOBAL_CB_BINDING_SLOT)
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
    float _PaddingGB2;
};
#endif

#if defined(LEB_CB_BINDING_SLOT)
cbuffer _LEBCB : register(LEB_CB_BINDING_SLOT)
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
#endif

#endif // CONSTANT_BUFFERS_HLSL