// Internal includes
#include "graphics/dx12_backend.h"
#include "render_pipeline/grid_renderer.h"
#include "math/operators.h"
#include "tools/shader_utils.h"

// SPLIT Threshold
#define SPLIT_THRESHOLD 2147483648

GridRenderer::GridRenderer()
{
}

GridRenderer::~GridRenderer()
{
}

void GridRenderer::initialize(GraphicsDevice device)
{
    // Keep track of the device
    m_Device = device;

    // Constant buffers
    m_GridCB = d3d12::resources::create_constant_buffer(m_Device, sizeof(GridCB), ConstantBufferType::Mixed);
    m_LinearClampSampler = d3d12::resources::create_sampler(m_Device, { FilterMode::Linear, SamplerMode::Clamp, SamplerMode::Clamp, SamplerMode::Clamp, 1 });
}

void GridRenderer::release()
{
    // Destroy the shaders
    d3d12::graphics_pipeline::destroy_graphics_pipeline(m_RasterizerGP);
    d3d12::compute_shader::destroy_compute_shader(m_InsideDensityCS);
    d3d12::compute_shader::destroy_compute_shader(m_OutsideDensityCS);
    d3d12::compute_shader::destroy_compute_shader(m_InsidePTCS);
    d3d12::compute_shader::destroy_compute_shader(m_OutsidePTCS);
    d3d12::resources::destroy_graphics_buffer(m_DensityBuffer[0]);
    if(m_DensityBuffer[1])
        d3d12::resources::destroy_graphics_buffer(m_DensityBuffer[1]);
    d3d12::resources::destroy_constant_buffer(m_GridCB);
    d3d12::resources::destroy_sampler(m_LinearClampSampler);
}

void GridRenderer::reload_shaders(const std::string& shaderLibrary)
{
    ComputeShaderDescriptor csd;
    csd.includeDirectories.push_back(shaderLibrary);
    csd.defines = m_ShaderDefines;

    // Density
    {
        csd.filename = shaderLibrary + "\\Grid\\Density.compute";

        csd.kernelname = "InsideVolumeIntegrator";
        compile_and_replace_compute_shader(m_Device, csd, m_InsideDensityCS);

        csd.kernelname = "OutsideVolumeIntegrator";
        compile_and_replace_compute_shader(m_Device, csd, m_OutsideDensityCS);
    }

    // PT
    {
        csd.filename = shaderLibrary + "\\Grid\\ForwardPT.compute";

        csd.kernelname = "InsideVolumeIntegrator";
        compile_and_replace_compute_shader(m_Device, csd, m_InsidePTCS);

        csd.kernelname = "OutsideVolumeIntegrator";
        compile_and_replace_compute_shader(m_Device, csd, m_OutsidePTCS);
    }

    {
        // Create the graphics pipeline
        GraphicsPipelineDescriptor gpd;
        gpd.filename = shaderLibrary + "\\Grid\\Rasterizer.graphics";
        gpd.includeDirectories.push_back(shaderLibrary);
        gpd.depthStencilState.enableDepth = true;
        gpd.depthStencilState.depthtest = DepthTest::Less;
        gpd.depthStencilState.depthWrite = false;
        gpd.cullMode = CullMode::None;
        gpd.geometryKernelName = "geom";

        BlendState blend;
        blend.enableBlend = true;
        blend.SrcBlend = BlendFactor::One;
        blend.DestBlend = BlendFactor::One;
        blend.BlendOp = BlendOperator::Add;

        gpd.blendState = blend;

        compile_and_replace_graphics_pipeline(m_Device, gpd, m_RasterizerGP);
    }
}

void GridRenderer::load_geometry(const std::string& filePath)
{
    // Import the volume
    grid_volume::import_grid_volume(filePath.c_str(), m_Volume);
    m_NumCells = m_Volume.densityArray.size();
    m_SplitBuffer = m_NumCells * sizeof(float) > SPLIT_THRESHOLD;

    // Create the runtime buffers
    if (m_SplitBuffer)
    {
        m_DensityBuffer[0] = d3d12::resources::create_graphics_buffer(m_Device, SPLIT_THRESHOLD, sizeof(float), GraphicsBufferType::Default);
        m_DensityBuffer[1] = d3d12::resources::create_graphics_buffer(m_Device, m_NumCells * sizeof(float) - SPLIT_THRESHOLD, sizeof(float), GraphicsBufferType::Default);
    }
    else
    {
        m_DensityBuffer[0] = d3d12::resources::create_graphics_buffer(m_Device, m_NumCells * sizeof(float), sizeof(float), GraphicsBufferType::Default);
        m_DensityBuffer[1] = 0;
    }
}

void GridRenderer::upload_geometry(CommandQueue cmdQ, CommandBuffer cmdB)
{
    // Buffer Size
    uint64_t bufferSize = m_NumCells * sizeof(float);

    // More than the threshold
    if (bufferSize > SPLIT_THRESHOLD)
    {
        // How many uploads
        const uint64_t uploadBufferSize = SPLIT_THRESHOLD;
        const uint32_t numUploadRounds = (uint32_t)((bufferSize + uploadBufferSize - 1) / uploadBufferSize);

        // Create the upload buffer
        GraphicsBuffer uploadBuffer = d3d12::resources::create_graphics_buffer(m_Device, uploadBufferSize, sizeof(float), GraphicsBufferType::Upload);

        // Upload the density
        for (uint32_t upIdx = 0; upIdx < numUploadRounds; ++upIdx)
        {
            // Set the CPU data
            uint64_t memoryToUpload = std::min(uploadBufferSize, bufferSize);
            d3d12::resources::set_buffer_data(uploadBuffer, (const char*)m_Volume.densityArray.data() + uploadBufferSize * upIdx, memoryToUpload);

            // Reset the command buffer
            d3d12::command_buffer::reset(cmdB);

            // Copy the upload buffers
            d3d12::command_buffer::copy_graphics_buffer(cmdB, uploadBuffer, 0, m_DensityBuffer[upIdx], 0, memoryToUpload);

            // Close and flush the command buffer
            d3d12::command_buffer::close(cmdB);
            d3d12::command_queue::execute_command_buffer(cmdQ, cmdB);
            d3d12::command_queue::flush(cmdQ);

            // Substract the uploaded memory
            bufferSize -= uploadBufferSize;
        }

        // Destroy the temporary resources
        d3d12::resources::destroy_graphics_buffer(uploadBuffer);
    }
    else
    {
        // Create the runtime buffers
        GraphicsBuffer densityBufferUp = d3d12::resources::create_graphics_buffer(m_Device, m_NumCells * sizeof(float), sizeof(float), GraphicsBufferType::Upload);
        d3d12::resources::set_buffer_data(densityBufferUp, (const char*)m_Volume.densityArray.data(), m_NumCells * sizeof(float));

        // Reset the command buffer
        d3d12::command_buffer::reset(cmdB);

        // Copy the upload buffers
        d3d12::command_buffer::copy_graphics_buffer(cmdB, densityBufferUp, m_DensityBuffer[0]);

        // Close and flush the command buffer
        d3d12::command_buffer::close(cmdB);
        d3d12::command_queue::execute_command_buffer(cmdQ, cmdB);
        d3d12::command_queue::flush(cmdQ);

        // Destroy the temporary resources
        d3d12::resources::destroy_graphics_buffer(densityBufferUp);
    }
}

void GridRenderer::upload_constant_buffers(CommandBuffer cmdB)
{
    GridCB gridCB;
    gridCB._GridResolution = m_Volume.resolution;
    gridCB._GridMaxPosition = float3({0.5, 0.5, 0.5});
    gridCB._GridMinPosition = float3({ -0.5, -0.5, -0.5 });
    gridCB._GridScale = rcp(m_Volume.scale);
    d3d12::resources::set_constant_buffer(m_GridCB, (const char*)&gridCB, sizeof(GridCB));
    d3d12::command_buffer::upload_constant_buffer(cmdB, m_GridCB);
}

void GridRenderer::render_volume(CommandBuffer cmd, ConstantBuffer globalCB, RenderTexture colorRT, RenderTexture depthRT, RenderingMode mode, Sky& sky, const Camera& camera)
{
    // Evaluate if we're inside or outside
    bool outsideCamera = camera.position.x >= 0.5 * m_Volume.scale.x 
                        || camera.position.x <= -0.5 * m_Volume.scale.x 
                        || camera.position.y >= 0.5 * m_Volume.scale.y
                        || camera.position.y <= -0.5 * m_Volume.scale.y
                        || camera.position.z >= 0.5 * m_Volume.scale.z
                        || camera.position.z <= -0.5 * m_Volume.scale.z;

    // Get the texture dimensions
    uint32_t width, height, depth;
    d3d12::resources::render_texture_dimensions(colorRT, width, height, depth);

    // Render
    switch (mode)
    {
        case RenderingMode::DebugView:
        {
            d3d12::command_buffer::set_render_texture(cmd, colorRT, depthRT);
            d3d12::command_buffer::set_viewport(cmd, 0, 0, width, height);

            // CBVs
            d3d12::command_buffer::set_graphics_pipeline_cbuffer(cmd, m_RasterizerGP, "_GlobalCB", globalCB);
            d3d12::command_buffer::set_graphics_pipeline_cbuffer(cmd, m_RasterizerGP, "_GridCB", m_GridCB);

            // Density
            d3d12::command_buffer::set_graphics_pipeline_buffer(cmd, m_RasterizerGP, "_DensityBuffer0", m_DensityBuffer[0]);
            d3d12::command_buffer::set_graphics_pipeline_buffer(cmd, m_RasterizerGP, "_DensityBuffer1", m_DensityBuffer[0]);

            // Draw
            d3d12::command_buffer::draw_procedural(cmd, m_RasterizerGP, 12, (uint32_t)m_Volume.densityArray.size());
        }
        break;
        case RenderingMode::DensityIntegration:
        {
            if (outsideCamera)
            {
                // CBVs
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_OutsideDensityCS, "_GlobalCB", globalCB);
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_OutsideDensityCS, "_GridCB", m_GridCB);

                // SRVs
                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsideDensityCS, "_DensityBuffer0", m_DensityBuffer[0]);
                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsideDensityCS, "_DensityBuffer1", m_DensityBuffer[1]);

                // UAVs
                d3d12::command_buffer::set_compute_shader_render_texture(cmd, m_OutsideDensityCS, "_ColorTexture", colorRT);

                // Dispatch
                d3d12::command_buffer::dispatch(cmd, m_OutsideDensityCS, (width + 7) / 8, (height + 7) / 8, 1);
            }
            else
            {
                // CBVs
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_InsideDensityCS, "_GlobalCB", globalCB);
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_InsideDensityCS, "_GridCB", m_GridCB);

                // SRVs
                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_InsideDensityCS, "_DensityBuffer0", m_DensityBuffer[0]);
                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_InsideDensityCS, "_DensityBuffer1", m_DensityBuffer[1]);

                // UAVs
                d3d12::command_buffer::set_compute_shader_render_texture(cmd, m_InsideDensityCS, "_ColorTexture", colorRT);

                // Dispatch
                d3d12::command_buffer::dispatch(cmd, m_InsideDensityCS, (width + 7) / 8, (height + 7) / 8, 1);
            }
        }
        break;
        case RenderingMode::ForwardPT:
        {
            if (outsideCamera)
            {
                // CBVs
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_OutsidePTCS, "_GlobalCB", globalCB);
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_OutsidePTCS, "_SkyAtmosphereCB", sky.constant_buffer());
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_OutsidePTCS, "_GridCB", m_GridCB);

                // SRVs
                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsidePTCS, "_DensityBuffer0", m_DensityBuffer[0]);
                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsidePTCS, "_DensityBuffer1", m_DensityBuffer[1]);
                d3d12::command_buffer::set_compute_shader_texture(cmd, m_OutsidePTCS, "_TransmittanceLUTTexture", sky.transmittance_lut());
                d3d12::command_buffer::set_compute_shader_texture(cmd, m_OutsidePTCS, "_MultiScatteringLUTTexture", sky.multi_scattering_lut());
                d3d12::command_buffer::set_compute_shader_sampler(cmd, m_OutsidePTCS, "_sampler_linear_clamp", m_LinearClampSampler);

                // UAVs
                d3d12::command_buffer::set_compute_shader_render_texture(cmd, m_OutsidePTCS, "_ColorTexture", colorRT);

                // Dispatch
                d3d12::command_buffer::dispatch(cmd, m_OutsidePTCS, (width + 7) / 8, (height + 7) / 8, 1);
            }
            else
            {
                // CBVs
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_InsidePTCS, "_GlobalCB", globalCB);
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_InsidePTCS, "_SkyAtmosphereCB", sky.constant_buffer());
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_InsidePTCS, "_GridCB", m_GridCB);

                // SRVs
                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_InsidePTCS, "_DensityBuffer0", m_DensityBuffer[0]);
                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_InsidePTCS, "_DensityBuffer1", m_DensityBuffer[1]);
                d3d12::command_buffer::set_compute_shader_texture(cmd, m_InsidePTCS, "_TransmittanceLUTTexture", sky.transmittance_lut());
                d3d12::command_buffer::set_compute_shader_texture(cmd, m_InsidePTCS, "_MultiScatteringLUTTexture", sky.multi_scattering_lut());
                d3d12::command_buffer::set_compute_shader_sampler(cmd, m_InsidePTCS, "_sampler_linear_clamp", m_LinearClampSampler);

                // UAVs
                d3d12::command_buffer::set_compute_shader_render_texture(cmd, m_InsidePTCS, "_ColorTexture", colorRT);

                // Dispatch
                d3d12::command_buffer::dispatch(cmd, m_InsidePTCS, (width + 7) / 8, (height + 7) / 8, 1);
            }
        }
        break;
    }
}