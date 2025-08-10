#pragma once

// Includes
#include "graphics/types.h"
#include "graphics/descriptors.h"
#include "render_pipeline/constant_buffers.h"
#include "render_pipeline/camera.h"
#include "render_pipeline/rendering_mode.h"
#include "render_pipeline/sky.h"
#include "volume/grid_volume.h"

// System includes
#include <string>

class GridRenderer
{
public:
    // Cst & Dst
    GridRenderer();
    ~GridRenderer();

    // Initialization and release
    void initialize(GraphicsDevice device);
    void release();

    // Reloading
    void load_geometry(const std::string& filePath);
    void reload_shaders(const std::string& shaderLibrary);
    void upload_geometry(CommandQueue cmdQ, CommandBuffer cmdB);

    // Rendering
    void upload_constant_buffers(CommandBuffer cmdB);
    void render_volume(CommandBuffer cmdB, ConstantBuffer globalCB, RenderTexture colorRT, RenderTexture depthRT, RenderingMode mode, Sky& sky, const Camera& camera);

    // Resources
    ConstantBuffer grid_cb() const { return m_GridCB; }

protected:
    // Graphics Device
    GraphicsDevice m_Device = 0;

    // CPU resources
    GridVolume m_Volume = GridVolume();
    uint64_t m_NumCells = 0;
    bool m_SplitBuffer = false;
    std::vector<std::string> m_ShaderDefines;
    Sampler m_LinearClampSampler = 0;

    // GPU Resources
    ConstantBuffer m_GridCB = 0;
    GraphicsBuffer m_DensityBuffer[2] = { 0, 0 };

    // Shaders
    GraphicsPipeline m_RasterizerGP = 0;
    ComputeShader m_InsideDensityCS = 0;
    ComputeShader m_OutsideDensityCS = 0;
    ComputeShader m_InsidePTCS = 0;
    ComputeShader m_OutsidePTCS = 0;
};