#pragma once

// SDK includes
#include "graphics/types.h"
#include "volume/leb_volume_gpu.h"

// System includes
#include <string>

class FrustumRenderer
{
public:
    // Cst & Dst
    FrustumRenderer();
    ~FrustumRenderer();

    // Init and release
    void initialize(GraphicsDevice device, const LEBVolumeGPU& volume);
    void release();

    // Reload the shaders
    void reload_shader(const std::string& shaderLibrary);

    // Rendering
    void upload_constant_buffers(CommandBuffer cmdB);
    void render_above(CommandBuffer cmdB, ConstantBuffer globalCB);
    void render_under(CommandBuffer cmdB, ConstantBuffer globalCB);

private:
    // Device
    GraphicsDevice m_Device = 0;

    // Graphics pipeline
    GraphicsPipeline m_FrustumAboveGP = 0;
    GraphicsPipeline m_FrustumUnderGP = 0;
    ConstantBuffer m_UpdateCB = 0;
    float3 m_Position = { 0.0, 0.0, 0.0 };
    float4x4 m_ViewProj = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    float3 m_VolumeMin = { 0.0, 0.0, 0.0 };
    float3 m_VolumeMax = { 0.0, 0.0, 0.0 };
};