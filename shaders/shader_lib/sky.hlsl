#ifndef SKY_HLSL
#define SKY_HLSL

// Sky includes
#include "shader_lib/sky_utilities.hlsl"

float3 sky_color(float3 rayDir)
{
    // We need to ensure that this is never lower than a certain radius.
    float3 luminance, throughput;
    IntegrateLuminanceThroughput(float3(0.0, 6371.0, 0.0), rayDir.xyz, _SunDirection, 0.0, 4, _TransmittanceLUTTexture, _MultiScatteringLUTTexture, _sampler_linear_clamp, luminance, throughput);
    return luminance * _SkyIntensity;
}

float3 sun_color()
{
    float viewZenithCosAngle = dot(_SunDirection, float3(0, 1, 0));
    float2 uv;
    LutTransmittanceParamsToUv(6371.0, viewZenithCosAngle, uv);
    return _TransmittanceLUTTexture.SampleLevel(_sampler_linear_clamp, uv, 0).rgb * _SunIntensity;
}

#endif // SKY_HLSL