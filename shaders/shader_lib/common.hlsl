#ifndef COMMON_H
#define COMMON_H

// Generic constants
#define PI 3.14159265358979323846
#define HALF_PI (PI / 2.0)
#define TWO_PI (PI * 2.0)
#define FLT_EPSILON 1.192092896e-07
#define INV_PI (1.0 /  PI)
#define FLT_MIN 1.175494351e-38
#define FLT_MAX 3.402823466e+38
#define PHI 1.61803398874989484820459
#define UINT32_MAX 0xffffffff

// Binding slot helpers
#define CBV_SLOT(NUM_SLOT) b##NUM_SLOT
#define SRV_SLOT(NUM_SLOT) t##NUM_SLOT
#define UAV_SLOT(NUM_SLOT) u##NUM_SLOT
#define SPL_SLOT(NUM_SLOT) s##NUM_SLOT

// NDC to clip Space
float4 evaluate_clip_space_position(float2 positionNDC, float depthValue)
{
    float4 positionCS = float4(positionNDC * 2.0 - 1.0, depthValue, 1.0);
    positionCS.y = -positionCS.y;
    return positionCS;
}

// NDC to RWS
float3 evaluate_world_space_position(float2 positionNDC, float depthValue, float4x4 invViewProjMatrix)
{
    float4 positionCS  = evaluate_clip_space_position(positionNDC, depthValue);
    float4 hpositionWS = mul(invViewProjMatrix, positionCS);
    return hpositionWS.xyz / hpositionWS.w;
}

float2 clip_space_to_pixel(float4 positionCS, float2 screenSize)
{
    float2 p = positionCS.xy / positionCS.w;
    p.y = -p.y;
    p.xy = (p.xy * 0.5 + 0.5);
    p.xy *= screenSize.xy;
    return p.xy;
}

float3 sample_sphere(float2 randomSample)
{
    // Convert randomSample from [0,1] to [-1,1] for uniform hemisphere mapping
    float z = randomSample.x * 2.0 - 1.0; // Z coordinate in [-1,1]
    float phi = randomSample.y * TWO_PI; // Azimuthal angle in [0, 2*PI]

    // Calculate radius of the circle at height z
    float radius = sqrt(-(z * z - 1.0));

    // Convert spherical to Cartesian coordinates
    float x = radius * cos(phi);
    float y = radius * sin(phi);

    return float3(x, y, z);
}

float luminance(float3 color)
{
    return 0.2126 * color.x + 0.7152* color.y + 0.0722 * color.z;
}

float fast_tonemap(float color)
{
    return color / (1.0 + color);
}

float3 fast_tonemap(float3 color)
{
    color /= (1.0 + color);
    return color;
}

float3 tonemap(float3 color)
{
    float lum = luminance(color);
    return color /  (1.0 + lum);
}

float4 fast_tonemap(float4 color)
{
    color.xyz /= (1.0 + color.xyz);
    return color;
}

float3 transform_dir(float3 dir, float3 scale)
{
    return normalize(dir * scale);
}

float3 transform_dir_inv(float3 dir, float3 scale)
{
    return normalize(dir / scale);
}

#include "shader_lib/rand.hlsl"

#define NUM_MAX_SEGMENTS 1024

#endif
