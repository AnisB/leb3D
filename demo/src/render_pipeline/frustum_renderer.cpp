// Internal includes
#include "graphics/dx12_backend.h"
#include "render_pipeline/constant_buffers.h"
#include "render_pipeline/frustum_renderer.h"
#include "math/operators.h"
#include "tools/shader_utils.h"

// Cst & Dst
FrustumRenderer::FrustumRenderer()
{
}

FrustumRenderer::~FrustumRenderer()
{
}

// Init and release
void FrustumRenderer::initialize(GraphicsDevice device, const LEBVolumeGPU& volume)
{
	m_Device = device;
	m_UpdateCB = d3d12::resources::create_constant_buffer(m_Device, sizeof(UpdateCB), ConstantBufferType::Mixed);
	m_Position = volume.cameraPosition;
	m_ViewProj = volume.vpMat;
	m_VolumeMin = volume.scale * float3(-0.5, -0.5, -0.5);
	m_VolumeMax = volume.scale * float3(0.5, 0.5, 0.5);
}

void FrustumRenderer::release()
{
	d3d12::graphics_pipeline::destroy_graphics_pipeline(m_FrustumAboveGP);
	d3d12::graphics_pipeline::destroy_graphics_pipeline(m_FrustumUnderGP);
	d3d12::resources::destroy_constant_buffer(m_UpdateCB);
}

// Reload the shaders
void FrustumRenderer::reload_shader(const std::string& shaderLibrary)
{
    GraphicsPipelineDescriptor gpd;
    gpd.includeDirectories.push_back(shaderLibrary);
	gpd.filename = shaderLibrary + "\\Frustum.graphics";
	gpd.geometryKernelName = "geom";
	gpd.depthStencilState.enableDepth = true;
	gpd.depthStencilState.depthtest = DepthTest::AlwaysPass;
	gpd.depthStencilState.depthWrite = false;
	gpd.depthStencilState.depthStencilFormat = TextureFormat::Depth32Stencil8;
	gpd.cullMode = CullMode::None;
	gpd.blendState.enableBlend = true;
	gpd.blendState.SrcBlend = BlendFactor::SrcAlpha;
	gpd.blendState.DestBlend = BlendFactor::InvSrcAlpha;
	gpd.blendState.SrcBlendAlpha = BlendFactor::Zero;
	gpd.blendState.DestBlendAlpha = BlendFactor::One;
	gpd.blendState.BlendOp = BlendOperator::Add;

	{
		gpd.fragmentKernelName = "frag_above";
		compile_and_replace_graphics_pipeline(m_Device, gpd, m_FrustumAboveGP);

		gpd.fragmentKernelName = "frag_under";
		compile_and_replace_graphics_pipeline(m_Device, gpd, m_FrustumUnderGP);
	}
}

void FrustumRenderer::upload_constant_buffers(CommandBuffer cmdB)
{
	// Global constant buffer
	UpdateCB updateCB;
	updateCB._UpdateCameraPosition = m_Position;
	updateCB._UpdateViewProjectionMatrix = m_ViewProj;
	updateCB._UpdateInvViewProjectionMatrix = inverse(updateCB._UpdateViewProjectionMatrix);
	updateCB._UpdateFarPlaneDistance = 2.5;
	updateCB._UpdateFOV = 30.0 * DEG_TO_RAD;
	updateCB._VolumeMinPosition = m_VolumeMin;
	updateCB._VolumeMaxPosition = m_VolumeMax;
	d3d12::resources::set_constant_buffer(m_UpdateCB, (const char*)&updateCB, sizeof(UpdateCB));
	d3d12::command_buffer::upload_constant_buffer(cmdB, m_UpdateCB);
}

void FrustumRenderer::render_above(CommandBuffer cmdB, ConstantBuffer globalCB)
{
	d3d12::command_buffer::start_section(cmdB, "Render Frustum");
	{
		// CBVs
		d3d12::command_buffer::set_graphics_pipeline_cbuffer(cmdB, m_FrustumAboveGP, "_GlobalCB", globalCB);
		d3d12::command_buffer::set_graphics_pipeline_cbuffer(cmdB, m_FrustumAboveGP, "_UpdateCB", m_UpdateCB);

		// Draw a full screen quad
		d3d12::command_buffer::draw_procedural(cmdB, m_FrustumAboveGP, 4, 1);
	}
	d3d12::command_buffer::end_section(cmdB);
}

void FrustumRenderer::render_under(CommandBuffer cmdB, ConstantBuffer globalCB)
{
	d3d12::command_buffer::start_section(cmdB, "Render Frustum");
	{
		// CBVs
		d3d12::command_buffer::set_graphics_pipeline_cbuffer(cmdB, m_FrustumUnderGP, "_GlobalCB", globalCB);
		d3d12::command_buffer::set_graphics_pipeline_cbuffer(cmdB, m_FrustumUnderGP, "_UpdateCB", m_UpdateCB);

		// Draw a full screen quad
		d3d12::command_buffer::draw_procedural(cmdB, m_FrustumUnderGP, 4, 1);
	}
	d3d12::command_buffer::end_section(cmdB);
}