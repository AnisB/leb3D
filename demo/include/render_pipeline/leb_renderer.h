#pragma once

// Includes
#include "graphics/types.h"
#include "graphics/descriptors.h"
#include "render_pipeline/constant_buffers.h"
#include "render_pipeline/rendering_mode.h"
#include "render_pipeline/camera.h"
#include "render_pipeline/morton_cache.h"
#include "render_pipeline/sky.h"
#include "render_pipeline/grid_renderer.h"
#include "volume/leb_volume_gpu.h"

// System includes
#include <string>

class LEBRenderer
{
public:
    // Cst & Dst
    LEBRenderer();
    ~LEBRenderer();

    // Initialization and release
    void initialize(GraphicsDevice device, uint2 screenRes);
    void release();

    // Reloading
    void load_geometry(const std::string& filePath);
    void reload_shaders(const std::string& shaderLibrary);
    void upload_geometry(CommandQueue cmdQ, CommandBuffer cmdB);
    LEBVolumeGPU& volume() { return m_Volume; }

    // Rendering
    void upload_constant_buffers(CommandBuffer cmdB, const float3& cameraPosition);
    void render_volume(CommandBuffer cmdB, ConstantBuffer globalCB, 
                        RenderTexture colorRT, RenderTexture depthRT,
                        RenderingMode mode, Sky& sky, const Camera& camera);

private:
    // Build the RTAS
    void build_rtas(CommandQueue cmdQ, CommandBuffer cmdB);

    // Build the morton cache
    void build_morton_cache();

private:
    // Graphics device
    GraphicsDevice m_Device = 0;

    // Resources
    uint32_t m_NumTetrahedron = 0;
    bool m_SplitBuffer = false;

    // Volume CPU data
    LEBVolumeGPU m_Volume = LEBVolumeGPU();
    MortonCache m_MortonCache = MortonCache();
    std::vector<std::string> m_ShaderDefines;
    Sampler m_LinearClampSampler = 0;
    uint32_t m_NumOutsideElements = 0;

    // LEB structure
    GraphicsBuffer m_TetraDataBuffer[2] = { 0, 0 };
    GraphicsBuffer m_DirectionBuffer = 0;
    GraphicsBuffer m_PositionBuffer = 0;

    // RTAS
    GraphicsBuffer m_RTASIndexBuffer = 0;
    GraphicsBuffer m_RTASPositionBuffer = 0;
    GraphicsBuffer m_ElementIndexBuffer = 0;
    BottomLevelAS m_BLAS = 0;
    TopLevelAS m_TLAS = 0;

    // Runtime buffers 
    ConstantBuffer m_LEBCB = 0;
    GraphicsBuffer m_PrimitiveBuffer = 0;
    GraphicsBuffer m_DistanceBuffer = 0;

    // Shaders
    ComputeShader m_IntersectBVHCS = 0;
        // Density
    ComputeShader m_InsideDensityCS = 0;
    ComputeShader m_OutsideDensityCS = 0;
        // PT
    ComputeShader m_InsidePTCS = 0;
    ComputeShader m_OutsidePTCS = 0;
        // Debug
    GraphicsPipeline m_DrawVolumeGP = 0;
};