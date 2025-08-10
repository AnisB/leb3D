// Internal includes
#include "graphics/dx12_backend.h"
#include "render_pipeline/leb_renderer.h"
#include "tools/shader_utils.h"
#include "tools/security.h"
#include "math/operators.h"

// External includes
#include <iostream>

// List of possible directions
#define NUM_DIRECTIONS 9
#define INV_SQRT2 0.70710678118
#define SPLIT_COUNT_THRESHOLD 100663296
#define SPLIT_SIZE_THRESHOLD (SPLIT_COUNT_THRESHOLD * sizeof(TetraData))
const float3 g_DirectionsRaw[NUM_DIRECTIONS] = { {-1, 0, 0},
                                        {-INV_SQRT2, -INV_SQRT2, 0},
                                        {-INV_SQRT2, -0, -INV_SQRT2},
                                        {-INV_SQRT2, 0, INV_SQRT2},
                                        {-INV_SQRT2, INV_SQRT2, 0},
                                        {0, -1, 0},
                                        {0, -INV_SQRT2, -INV_SQRT2},
                                        {0, -INV_SQRT2, INV_SQRT2},
                                        {0, 0, -1},
};

LEBRenderer::LEBRenderer()
{
}

LEBRenderer::~LEBRenderer()
{
}

void LEBRenderer::initialize(GraphicsDevice device, uint2 screenRes)
{
	// Keep track of the device
	m_Device = device;

    // Constant buffers
    m_LEBCB = d3d12::resources::create_constant_buffer(m_Device, sizeof(LEBCB), ConstantBufferType::Mixed);

    // Screen resolution
    m_PrimitiveBuffer = d3d12::resources::create_graphics_buffer(m_Device, screenRes.x * screenRes.y * sizeof(uint32_t), sizeof(uint32_t), GraphicsBufferType::Default);
    m_DistanceBuffer = d3d12::resources::create_graphics_buffer(m_Device, screenRes.x * screenRes.y * sizeof(float), sizeof(float), GraphicsBufferType::Default);
    m_LinearClampSampler = d3d12::resources::create_sampler(m_Device, { FilterMode::Linear, SamplerMode::Clamp, SamplerMode::Clamp, SamplerMode::Clamp, 1 });
}

void LEBRenderer::release()
{
    // Resources
    d3d12::resources::destroy_graphics_buffer(m_TetraDataBuffer[0]);
    if (m_TetraDataBuffer[1])
        d3d12::resources::destroy_graphics_buffer(m_TetraDataBuffer[1]);
    d3d12::resources::destroy_graphics_buffer(m_DirectionBuffer);
    d3d12::resources::destroy_graphics_buffer(m_PositionBuffer);

    // Runtime resources
    d3d12::resources::destroy_constant_buffer(m_LEBCB);
    d3d12::resources::destroy_graphics_buffer(m_PrimitiveBuffer);
    d3d12::resources::destroy_graphics_buffer(m_DistanceBuffer);
    d3d12::resources::destroy_sampler(m_LinearClampSampler);

    // Destroy the shaders
    d3d12::compute_shader::destroy_compute_shader(m_IntersectBVHCS);
    d3d12::compute_shader::destroy_compute_shader(m_InsideDensityCS);
    d3d12::compute_shader::destroy_compute_shader(m_OutsideDensityCS);
    d3d12::compute_shader::destroy_compute_shader(m_InsidePTCS);
    d3d12::compute_shader::destroy_compute_shader(m_OutsidePTCS);
    d3d12::graphics_pipeline::destroy_graphics_pipeline(m_DrawVolumeGP);

    // RTAS
    d3d12::resources::destroy_graphics_buffer(m_RTASIndexBuffer);
    d3d12::resources::destroy_graphics_buffer(m_RTASPositionBuffer);
    d3d12::resources::destroy_graphics_buffer(m_ElementIndexBuffer);
    d3d12::resources::destroy_blas(m_BLAS);
    d3d12::resources::destroy_tlas(m_TLAS);
}

void LEBRenderer::reload_shaders(const std::string& shaderLibrary)
{
    // Common stuff
    ComputeShaderDescriptor csd;
    csd.includeDirectories.push_back(shaderLibrary);
    csd.defines = m_ShaderDefines;

    {
        csd.filename = shaderLibrary + "\\LEB\\IntersectBVH.compute";
        csd.kernelname = "IntersectBVH";
        compile_and_replace_compute_shader(m_Device, csd, m_IntersectBVHCS);
    }

    {
        csd.filename = shaderLibrary + "\\LEB\\Density.compute";
        csd.kernelname = "InsideVolumeIntegrator";
        compile_and_replace_compute_shader(m_Device, csd, m_InsideDensityCS);

        csd.kernelname = "OutsideVolumeIntegrator";
        compile_and_replace_compute_shader(m_Device, csd, m_OutsideDensityCS);
    }

    {
        csd.filename = shaderLibrary + "\\LEB\\ForwardPT.compute";
        csd.kernelname = "InsideVolumeIntegrator";
        compile_and_replace_compute_shader(m_Device, csd, m_InsidePTCS);

        csd.kernelname = "OutsideVolumeIntegrator";
        compile_and_replace_compute_shader(m_Device, csd, m_OutsidePTCS);
    }

    {
        // Create the graphics pipeline
        GraphicsPipelineDescriptor gpd;
        gpd.defines = m_ShaderDefines;
        gpd.includeDirectories.push_back(shaderLibrary);
        gpd.filename = shaderLibrary + "\\LEB\\Rasterizer.graphics";
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
        compile_and_replace_graphics_pipeline(m_Device, gpd, m_DrawVolumeGP);
    }
}
void LEBRenderer::load_geometry(const std::string& filePath)
{
    // Import the volume
    leb_volume::import_leb_volume_gpu(filePath.c_str(), m_Volume);

    // Grab the number of elements and validate the size
    m_NumTetrahedron = (uint32_t)m_Volume.tetraData.size();
    m_SplitBuffer = m_NumTetrahedron > SPLIT_COUNT_THRESHOLD;

    // Build the morton codes
    build_morton_cache();

    // Create the runtime buffers
    if (m_SplitBuffer)
    {
        m_TetraDataBuffer[0] = d3d12::resources::create_graphics_buffer(m_Device, SPLIT_SIZE_THRESHOLD, sizeof(TetraData), GraphicsBufferType::Default);
        m_TetraDataBuffer[1] = d3d12::resources::create_graphics_buffer(m_Device, m_NumTetrahedron * sizeof(TetraData) - SPLIT_SIZE_THRESHOLD, sizeof(TetraData), GraphicsBufferType::Default);
    }
    else
    {
        m_TetraDataBuffer[0] = d3d12::resources::create_graphics_buffer(m_Device, m_NumTetrahedron * sizeof(TetraData), sizeof(TetraData), GraphicsBufferType::Default);
        m_TetraDataBuffer[1] = 0;
    }

    m_DirectionBuffer = d3d12::resources::create_graphics_buffer(m_Device, NUM_DIRECTIONS * sizeof(float3), sizeof(float), GraphicsBufferType::Default);
    m_PositionBuffer = d3d12::resources::create_graphics_buffer(m_Device, m_NumTetrahedron * 4 * sizeof(float3), sizeof(float3), GraphicsBufferType::Default);
}

void LEBRenderer::upload_geometry(CommandQueue cmdQ, CommandBuffer cmdB)
{
    // Tetra data
    uint64_t tetraDataBufferSize = m_NumTetrahedron * sizeof(TetraData);
    {
        // How many uploads
        const uint64_t uploadBufferSize = SPLIT_SIZE_THRESHOLD;
        const uint32_t numUploadRounds = (uint32_t)((tetraDataBufferSize + uploadBufferSize - 1) / uploadBufferSize);

        // Create the upload buffer
        GraphicsBuffer uploadBuffer = d3d12::resources::create_graphics_buffer(m_Device, uploadBufferSize, sizeof(TetraData), GraphicsBufferType::Upload);

        // Upload the density
        for (uint32_t upIdx = 0; upIdx < numUploadRounds; ++upIdx)
        {
            // Set the CPU data
            uint64_t memoryToUpload = std::min(uploadBufferSize, tetraDataBufferSize);
            d3d12::resources::set_buffer_data(uploadBuffer, (const char*)m_Volume.tetraData.data() + uploadBufferSize * upIdx, memoryToUpload);

            // Reset the command buffer
            d3d12::command_buffer::reset(cmdB);

            // Copy the upload buffers
            d3d12::command_buffer::copy_graphics_buffer(cmdB, uploadBuffer, 0, m_TetraDataBuffer[upIdx], 0, memoryToUpload);

            // Close and flush the command buffer
            d3d12::command_buffer::close(cmdB);
            d3d12::command_queue::execute_command_buffer(cmdQ, cmdB);
            d3d12::command_queue::flush(cmdQ);

            // Substract the uploaded memory
            tetraDataBufferSize -= uploadBufferSize;
        }

        // Destroy the temporary resources
        d3d12::resources::destroy_graphics_buffer(uploadBuffer);
    }

    // Directions
    GraphicsBuffer directionBufferUP = d3d12::resources::create_graphics_buffer(m_Device, NUM_DIRECTIONS * sizeof(float3), sizeof(float), GraphicsBufferType::Upload);
    d3d12::resources::set_buffer_data(directionBufferUP, (const char*)g_DirectionsRaw, NUM_DIRECTIONS * sizeof(float3));

    // Positions
    GraphicsBuffer positionBufferUp = d3d12::resources::create_graphics_buffer(m_Device, m_NumTetrahedron * 4 * sizeof(float3), sizeof(float3), GraphicsBufferType::Upload);
    d3d12::resources::set_buffer_data(positionBufferUp, (const char*)m_Volume.positionArray.data(), m_NumTetrahedron * 4 * sizeof(float3));

    // Reset the command buffer
    d3d12::command_buffer::reset(cmdB);

    // Copy the upload buffers
    d3d12::command_buffer::copy_graphics_buffer(cmdB, directionBufferUP, m_DirectionBuffer);
    d3d12::command_buffer::copy_graphics_buffer(cmdB, positionBufferUp, m_PositionBuffer);

    // Close and flush the command buffer
    d3d12::command_buffer::close(cmdB);
    d3d12::command_queue::execute_command_buffer(cmdQ, cmdB);
    d3d12::command_queue::flush(cmdQ);

    // Destroy the temporary resources
    d3d12::resources::destroy_graphics_buffer(directionBufferUP);
    d3d12::resources::destroy_graphics_buffer(positionBufferUp);

    // Build the RTAS
    build_rtas(cmdQ, cmdB);
}

void LEBRenderer::build_rtas(CommandQueue cmdQ, CommandBuffer cmdB)
{
    // How many outside elements?
    m_NumOutsideElements = (uint32_t)m_Volume.outsideElements.size();

    // RTAS Position buffer
    GraphicsBuffer posBufferUP = d3d12::resources::create_graphics_buffer(m_Device, m_NumOutsideElements * sizeof(float3) * 3, sizeof(float3), GraphicsBufferType::Upload);
    m_RTASPositionBuffer = d3d12::resources::create_graphics_buffer(m_Device, m_NumOutsideElements * sizeof(float3) * 3, sizeof(float3), GraphicsBufferType::Default);
    d3d12::resources::set_buffer_data(posBufferUP, (const char*)m_Volume.rtasPositionArray.data(), m_NumOutsideElements * 3 * sizeof(float3));

    // RTAS Index buffer
    GraphicsBuffer indexBufferUP = d3d12::resources::create_graphics_buffer(m_Device, m_NumOutsideElements * sizeof(uint3), sizeof(uint3), GraphicsBufferType::Upload);
    m_RTASIndexBuffer = d3d12::resources::create_graphics_buffer(m_Device, m_NumOutsideElements * sizeof(uint3), sizeof(uint3), GraphicsBufferType::Default);
    d3d12::resources::set_buffer_data(indexBufferUP, (const char*)m_Volume.rtasIndexArray.data(), m_NumOutsideElements * sizeof(uint3));

    // Element Index Buffer
    GraphicsBuffer elementIndexBufferUp = d3d12::resources::create_graphics_buffer(m_Device, m_NumOutsideElements * sizeof(uint32_t), sizeof(uint32_t), GraphicsBufferType::Upload);
    m_ElementIndexBuffer = d3d12::resources::create_graphics_buffer(m_Device, m_NumOutsideElements * sizeof(uint32_t), sizeof(uint32_t), GraphicsBufferType::Default);
    d3d12::resources::set_buffer_data(elementIndexBufferUp, (const char*)m_Volume.outsideElements.data(), m_NumOutsideElements * sizeof(uint32_t));

    // Create the acceleration structures
    m_BLAS = d3d12::resources::create_blas(m_Device, m_RTASPositionBuffer, m_NumOutsideElements * 3, m_RTASIndexBuffer, m_NumOutsideElements);
    m_TLAS = d3d12::resources::create_tlas(m_Device, 1);
    d3d12::resources::set_tlas_instance(m_TLAS, m_BLAS, 0);
    d3d12::resources::upload_tlas_instance_data(m_TLAS);

    // Reset the command buffer
    d3d12::command_buffer::reset(cmdB);

    // Copy the graphics buffers
    d3d12::command_buffer::copy_graphics_buffer(cmdB, posBufferUP, m_RTASPositionBuffer);
    d3d12::command_buffer::copy_graphics_buffer(cmdB, indexBufferUP, m_RTASIndexBuffer);
    d3d12::command_buffer::copy_graphics_buffer(cmdB, elementIndexBufferUp, m_ElementIndexBuffer);

    // Build the RTAS
    d3d12::command_buffer::build_blas(cmdB, m_BLAS);
    d3d12::command_buffer::build_tlas(cmdB, m_TLAS);

    // Close and flush the command buffer
    d3d12::command_buffer::close(cmdB);
    d3d12::command_queue::execute_command_buffer(cmdQ, cmdB);
    d3d12::command_queue::flush(cmdQ);

    // Destroy the temporary resources
    d3d12::resources::destroy_graphics_buffer(posBufferUP);
    d3d12::resources::destroy_graphics_buffer(indexBufferUP);
    d3d12::resources::destroy_graphics_buffer(elementIndexBufferUp);
}

void LEBRenderer::build_morton_cache()
{
    // Evaluate all the centers
    m_MortonCache.build_cache(m_Volume.centerArray.data(), m_NumTetrahedron);
}

void decompress_plane_equation(uint32_t cE, float3& planeDir, float& offsetToOrigin)
{
    const uint8_t normIdx = cE & 0xf;

    // Read it from shared memory
    planeDir = g_DirectionsRaw[normIdx];

    // Inverse if required
    planeDir = planeDir * ((cE & 0x10) ? -1.0f : 1.0f);

    // offset to origin
    const uint32_t offset = cE & 0xFFFFFFE0;
    offsetToOrigin = *reinterpret_cast<const float*>(&offset);
}

float ray_plane_intersection(const float3& rayOrigin, const float3& rayDirection, const float3& planeNormal, float planeOffset)
{
    float denom = dot(rayDirection, planeNormal);
    if (denom < 1e-6)
        return FLT_MAX;
    float t = -(dot(rayOrigin, planeNormal) + planeOffset);
    return t > -1e-6 ? t / denom : FLT_MAX;
}

float signed_distance(const float3& pos, uint32_t planeEquation)
{
    float3 normal;
    float offset;
    decompress_plane_equation(planeEquation, normal, offset);
    return (dot(pos, normal) + offset);
}

int point_inside_tetrahedron(float3 p, uint32_t pE0, uint32_t pE1, uint32_t pE2, uint32_t pE3)
{
    float sd0 = signed_distance(p, pE0);
    float sd1 = signed_distance(p, pE1);
    float sd2 = signed_distance(p, pE2);
    float sd3 = signed_distance(p, pE3);
    return (sd0 >= 0 && sd1 >= 0 && sd2 >= 0 && sd3 >= 0);
}

void LEBRenderer::upload_constant_buffers(CommandBuffer cmdB, const float3& cameraPosition)
{
    // Start filling the CB
    LEBCB lebCB;
    lebCB._NumTetrahedrons = (uint32_t)m_Volume.densityArray.size();
    lebCB._LEBScale = rcp(m_Volume.scale);

    // Initial primitive for our search
    uint32_t initialPrimitive = m_MortonCache.get_closest_element(cameraPosition * lebCB._LEBScale);

    // Is this the right primitive?
    const TetraData& initalData = m_Volume.tetraData[initialPrimitive];
    if (!point_inside_tetrahedron(cameraPosition, initalData.compressedEquations.x, initalData.compressedEquations.y, initalData.compressedEquations.z, initalData.compressedEquations.w))
    {
        // Ray origin, tetrahedron center
        float3 rayOrigin = m_Volume.centerArray[initialPrimitive];
        float3 segment = cameraPosition * lebCB._LEBScale - rayOrigin;
        float maxT = length(segment);
        float3 rayDir = segment / maxT;

        // March our structure till we reach the camera position
        uint32_t currentPrimitive = initialPrimitive;
        uint32_t prevPrimitive = UINT32_MAX;
        while (currentPrimitive != UINT32_MAX)
        {
            // Read the tetra data
            TetraData data = m_Volume.tetraData[currentPrimitive];

            // Max intersection
            float l = FLT_MAX;

            // Process the faces
            uint32_t candidate = UINT32_MAX;

            // Define our exit interface
            for (uint32_t faceIdx = 0; faceIdx < 4; ++faceIdx)
            {
                uint32_t faceNeighborIdx = at(data.neighbors, faceIdx);
                if (faceNeighborIdx == UINT32_MAX || (faceNeighborIdx != prevPrimitive))
                {
                    // Get the plane equation
                    float3 planeDir;
                    float offset;
                    decompress_plane_equation(at(data.compressedEquations, faceIdx), planeDir, offset);

                    // Intersect
                    float t = ray_plane_intersection(rayOrigin, rayDir, planeDir, offset);
                    if (t < l)
                    {
                        l = t;
                        candidate = faceNeighborIdx;
                    }
                }
            }

            if (maxT > l)
            {
                // Move to the next primitive
                prevPrimitive = currentPrimitive;
                currentPrimitive = candidate;
            }
            else
            {
                // Found it, done
                initialPrimitive = currentPrimitive;
                break;
            }
        }
    }

    lebCB._InitialPrimitive = initialPrimitive;
    d3d12::resources::set_constant_buffer(m_LEBCB, (const char*)&lebCB, sizeof(LEBCB));
    d3d12::command_buffer::upload_constant_buffer(cmdB, m_LEBCB);
}

void LEBRenderer::render_volume(CommandBuffer cmd, ConstantBuffer globalCB, RenderTexture colorRT, RenderTexture depthRT, RenderingMode mode, Sky& sky, const Camera& camera)
{
    // Num tetrahedrons
    const uint32_t numTetrahedrons = (uint32_t)m_Volume.densityArray.size();

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

    switch (mode)
    {
        case RenderingMode::DebugView:
        {
            // VP & RT
            d3d12::command_buffer::set_render_texture(cmd, colorRT, depthRT);
            d3d12::command_buffer::set_viewport(cmd, 0, 0, width, height);

            // CBVs
            d3d12::command_buffer::set_graphics_pipeline_cbuffer(cmd, m_DrawVolumeGP, "_GlobalCB", globalCB);
            d3d12::command_buffer::set_graphics_pipeline_cbuffer(cmd, m_DrawVolumeGP, "_LEBCB", m_LEBCB);

            // SRV
            d3d12::command_buffer::set_graphics_pipeline_buffer(cmd, m_DrawVolumeGP, "_PositionBuffer", m_PositionBuffer);
            d3d12::command_buffer::set_graphics_pipeline_buffer(cmd, m_DrawVolumeGP, "_TetraDataBuffer0", m_TetraDataBuffer[0]);
            d3d12::command_buffer::set_graphics_pipeline_buffer(cmd, m_DrawVolumeGP, "_TetraDataBuffer1", m_TetraDataBuffer[1]);

            // Draw
            d3d12::command_buffer::draw_procedural(cmd, m_DrawVolumeGP, 4, numTetrahedrons);
        }
        break;
        case RenderingMode::DensityIntegration:
        {
            if (outsideCamera)
            {
                {
                    // CBVs
                    d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_IntersectBVHCS, "_GlobalCB", globalCB);
                    d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_IntersectBVHCS, "_LEBCB", m_LEBCB);

                    // SRVs
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_IntersectBVHCS, "_ElementIndexBuffer", m_ElementIndexBuffer);
                    d3d12::command_buffer::set_compute_shader_rtas(cmd, m_IntersectBVHCS, "_InterfaceRTAS", m_TLAS);

                    // UAVs
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_IntersectBVHCS, "_PrimitiveBufferRW", m_PrimitiveBuffer);
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_IntersectBVHCS, "_DistanceBufferRW", m_DistanceBuffer);

                    // Dispatch
                    d3d12::command_buffer::dispatch(cmd, m_IntersectBVHCS, (width + 7) / 8, (height + 7) / 8, 1);
                    d3d12::command_buffer::uav_barrier_buffer(cmd, m_PrimitiveBuffer);
                }

                {
                    // CBVs
                    d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_OutsideDensityCS, "_GlobalCB", globalCB);
                    d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_OutsideDensityCS, "_LEBCB", m_LEBCB);

                    // SRVs
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsideDensityCS, "_TetraDataBuffer0", m_TetraDataBuffer[0]);
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsideDensityCS, "_TetraDataBuffer1", m_TetraDataBuffer[1]);
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsideDensityCS, "_PrimitiveBuffer", m_PrimitiveBuffer);
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsideDensityCS, "_DistanceBuffer", m_DistanceBuffer);
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsideDensityCS, "_DirectionBuffer", m_DirectionBuffer);

                    // UAVs
                    d3d12::command_buffer::set_compute_shader_render_texture(cmd, m_OutsideDensityCS, "_ColorTexture", colorRT);

                    // Dispatch
                    d3d12::command_buffer::dispatch(cmd, m_OutsideDensityCS, (width + 7) / 8, (height + 7) / 8, 1);
                }
            }
            else
            {
                // CBVs
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_InsideDensityCS, "_GlobalCB", globalCB);
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_InsideDensityCS, "_LEBCB", m_LEBCB);

                // SRVs
                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_InsideDensityCS, "_TetraDataBuffer0", m_TetraDataBuffer[0]);
                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_InsideDensityCS, "_TetraDataBuffer1", m_TetraDataBuffer[1]);
                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_InsideDensityCS, "_DistanceBuffer", m_DistanceBuffer);
                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_InsideDensityCS, "_DirectionBuffer", m_DirectionBuffer);

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
                {
                    // CBVs
                    d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_IntersectBVHCS, "_GlobalCB", globalCB);
                    d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_IntersectBVHCS, "_LEBCB", m_LEBCB);

                    // SRVs
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_IntersectBVHCS, "_ElementIndexBuffer", m_ElementIndexBuffer);
                    d3d12::command_buffer::set_compute_shader_rtas(cmd, m_IntersectBVHCS, "_InterfaceRTAS", m_TLAS);

                    // UAVs
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_IntersectBVHCS, "_PrimitiveBufferRW", m_PrimitiveBuffer);
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_IntersectBVHCS, "_DistanceBufferRW", m_DistanceBuffer);

                    // Dispatch
                    d3d12::command_buffer::dispatch(cmd, m_IntersectBVHCS, (width + 7) / 8, (height + 7) / 8, 1);
                    d3d12::command_buffer::uav_barrier_buffer(cmd, m_PrimitiveBuffer);
                }

                {
                    // CBVs
                    d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_OutsidePTCS, "_GlobalCB", globalCB);
                    d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_OutsidePTCS, "_SkyAtmosphereCB", sky.constant_buffer());
                    d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_OutsidePTCS, "_LEBCB", m_LEBCB);

                    // SRVs
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsidePTCS, "_TetraDataBuffer0", m_TetraDataBuffer[0]);
                    if (m_TetraDataBuffer[1] != 0)
                        d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsidePTCS, "_TetraDataBuffer1", m_TetraDataBuffer[1]);
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsidePTCS, "_PrimitiveBuffer", m_PrimitiveBuffer);
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsidePTCS, "_DistanceBuffer", m_DistanceBuffer);
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_OutsidePTCS, "_DirectionBuffer", m_DirectionBuffer);
                    d3d12::command_buffer::set_compute_shader_texture(cmd, m_OutsidePTCS, "_TransmittanceLUTTexture", sky.transmittance_lut());
                    d3d12::command_buffer::set_compute_shader_texture(cmd, m_OutsidePTCS, "_MultiScatteringLUTTexture", sky.multi_scattering_lut());

                    // Sampler
                    d3d12::command_buffer::set_compute_shader_sampler(cmd, m_OutsidePTCS, "_sampler_linear_clamp", m_LinearClampSampler);

                    // UAVs
                    d3d12::command_buffer::set_compute_shader_render_texture(cmd, m_OutsidePTCS, "_ColorTexture", colorRT);

                    // Dispatch
                    d3d12::command_buffer::dispatch(cmd, m_OutsidePTCS, (width + 7) / 8, (height + 7) / 8, 1);
                }
            }
            else
            {
                // CBVs
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_InsidePTCS, "_GlobalCB", globalCB);
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_InsidePTCS, "_SkyAtmosphereCB", sky.constant_buffer());
                d3d12::command_buffer::set_compute_shader_cbuffer(cmd, m_InsidePTCS, "_LEBCB", m_LEBCB);

                // SRVs
                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_InsidePTCS, "_TetraDataBuffer0", m_TetraDataBuffer[0]);
                if (m_TetraDataBuffer[1] != 0)
                    d3d12::command_buffer::set_compute_shader_buffer(cmd, m_InsidePTCS, "_TetraDataBuffer1", m_TetraDataBuffer[1]);

                d3d12::command_buffer::set_compute_shader_buffer(cmd, m_InsidePTCS, "_DirectionBuffer", m_DirectionBuffer);
                d3d12::command_buffer::set_compute_shader_texture(cmd, m_InsidePTCS, "_TransmittanceLUTTexture", sky.transmittance_lut());
                d3d12::command_buffer::set_compute_shader_texture(cmd, m_InsidePTCS, "_MultiScatteringLUTTexture", sky.multi_scattering_lut());

                // Sampler
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