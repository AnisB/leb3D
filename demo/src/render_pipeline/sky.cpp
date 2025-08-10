// Internal includes
#include "render_pipeline/sky.h"
#include "math/operators.h"
#include "graphics/dx12_backend.h"
#include "tools/shader_utils.h"

// Shader file and kernels
const char* sky_pre_compute_file = "SkyPreCompute.compute";
const char* transmittance_lut_kernel = "TransmittanceLUT";
const char* multi_scattering_lut_kernel = "MultiScatteringLUT";
const char* sky_view_lut_kernel = "SkyViewLUT";

// Resolution of the luts
const uint32_t transmittance_lut_width = 256;
const uint32_t transmittance_lut_height = 64;
const uint32_t multi_scat_lut_size = 32;
const uint32_t sky_view_lut_width = 192;
const uint32_t sky_view_lut_height = 108;

// CBVs
#define GLOBAL_CB_BINDING_SLOT CBV_SLOT(0)
#define SKY_ATMOSPHERE_BUFFER_SLOT CBV_SLOT(1)
#define RENDER_SKY_CBV_COUNT 2

// SRVs
#define TRANSMITTANCE_LUT_TEXTURE_SLOT SRV_SLOT(0)
#define MULTI_SCATTERING_LUT_TEXTURE_SLOT SRV_SLOT(1)
#define SKY_VIEW_LUT_TEXTURE_SLOT SRV_SLOT(2)

// Samplers
#define LINEAR_CLAMP_SAMPLER_SLOT SPL_SLOT(0)
#define RENDER_SKY_SPL_COUNT 4

struct SkyAtmosphereCB
{
	float3  _AbsorptionExtinction;
	float _PlanetRadiusSky;

	float3  _RayleighScattering;
	float   _MisePhaseFunctionG;

	float3  _MieScattering;
	float   _BottomRadius;

	float3  _MieExtinction;
	float   _TopRadius;

	float3  _MieAbsorption;
	float   _MiePhaseG;

	float3 _GroundAlbedo;
	float _PaddingSA0;

	float3 _PlanetCenterSky;
	float _PaddingSA1;

	float _RayleighDensity0LayerWidth;
	float _RayleighDensity0ExpTerm;
	float _RayleighDensity0ExpScale;
	float _RayleighDensity0LinearTerm;
	float _RayleighDensity0ConstantTerm;

	float _RayleighDensity1LayerWidth;
	float _RayleighDensity1ExpTerm;
	float _RayleighDensity1ExpScale;
	float _RayleighDensity1LinearTerm;
	float _RayleighDensity1ConstantTerm;

	float _MieDensity0LayerWidth;
	float _MieDensity0ExpTerm;
	float _MieDensity0ExpScale;
	float _MieDensity0LinearTerm;
	float _MieDensity0ConstantTerm;

	float _MieDensity1LayerWidth;
	float _MieDensity1ExpTerm;
	float _MieDensity1ExpScale;
	float _MieDensity1LinearTerm;
	float _MieDensity1ConstantTerm;

	float _AbsorptionDensity0LayerWidth;
	float _AbsorptionDensity0ExpTerm;
	float _AbsorptionDensity0ExpScale;
	float _AbsorptionDensity0LinearTerm;
	float _AbsorptionDensity0ConstantTerm;

	float _AbsorptionDensity1LayerWidth;
	float _AbsorptionDensity1ExpTerm;
	float _AbsorptionDensity1ExpScale;
	float _AbsorptionDensity1LinearTerm;
	float _AbsorptionDensity1ConstantTerm;
};

Sky::Sky()
{
}

Sky::~Sky()
{
}

void Sky::initialize(GraphicsDevice device)
{
	// Keep track of the device
	m_Device = device;

	// Buffers that hold the precomputations
	TextureDescriptor texDesc;
	texDesc.type = TextureType::Tex2D;
	texDesc.depth = 1;
	texDesc.mipCount = 1;
	texDesc.isUAV = true;
	texDesc.format = TextureFormat::R16G16B16A16_Float;
	texDesc.clearColor = float4({0.0, 0.0, 0.0, 0.0});

	// Create the transmittance lut
	texDesc.width = transmittance_lut_width;
	texDesc.height = transmittance_lut_height;
	m_TransmittanceLutTex = d3d12::resources::create_texture(device, texDesc);

	// Create the multi scattering lut
	texDesc.width = multi_scat_lut_size;
	texDesc.height = multi_scat_lut_size;
	m_MultiScatteringLutTex = d3d12::resources::create_texture(device, texDesc);

	// Create the sky view lut
	texDesc.width = sky_view_lut_width;
	texDesc.height = sky_view_lut_height;
	m_SkyViewLutTex = d3d12::resources::create_texture(device, texDesc);

	// Constant buffer
	m_SkyAtmosphereCB = d3d12::resources::create_constant_buffer(device, sizeof(SkyAtmosphereCB), ConstantBufferType::Mixed);

	// Create the sampler
	m_LinearClampSampler =  d3d12::resources::create_sampler(device, { FilterMode::Linear, SamplerMode::Clamp, SamplerMode::Clamp, SamplerMode::Clamp });
}

void Sky::release()
{
	d3d12::resources::destroy_constant_buffer(m_SkyAtmosphereCB);
	d3d12::compute_shader::destroy_compute_shader(m_MultiScatteringLutCS);
	d3d12::compute_shader::destroy_compute_shader(m_TransmittanceLutCS);

	d3d12::resources::destroy_texture(m_SkyViewLutTex);
	d3d12::resources::destroy_texture(m_MultiScatteringLutTex);
	d3d12::resources::destroy_texture(m_TransmittanceLutTex);

	d3d12::resources::destroy_sampler(m_LinearClampSampler);
}

void Sky::reload_shaders(const std::string& shaderLibrary)
{
	// Create the compute shaders
	ComputeShaderDescriptor csd;
	csd.includeDirectories.push_back(shaderLibrary);
	csd.filename = shaderLibrary + "\\Sky\\" + sky_pre_compute_file;

	// Transmittance Lut kernel
	csd.kernelname = transmittance_lut_kernel;
	compile_and_replace_compute_shader(m_Device, csd, m_TransmittanceLutCS);

	// Multi Scat Lut kernel
	csd.kernelname = multi_scattering_lut_kernel;
	compile_and_replace_compute_shader(m_Device, csd, m_MultiScatteringLutCS);
}

// Pre-rendering steps
void Sky::pre_render(CommandBuffer cmdB)
{
	d3d12::command_buffer::start_section(cmdB, "Prepare Sky");
	{
		// Update the constant buffers
		update_constant_buffer(cmdB);

		// Evaluate the transmittance LUT
		{
			// Constant buffers
			d3d12::command_buffer::set_compute_shader_cbuffer(cmdB, m_TransmittanceLutCS, "_SkyAtmosphereCB", m_SkyAtmosphereCB);

			// Output
			d3d12::command_buffer::set_compute_shader_texture(cmdB, m_TransmittanceLutCS, "_TransmittanceLUTTextureRW", m_TransmittanceLutTex);

			// Dispatch
			d3d12::command_buffer::dispatch(cmdB, m_TransmittanceLutCS, transmittance_lut_width / 8, transmittance_lut_height / 8, 1);

			// Barrier
			d3d12::command_buffer::uav_barrier_texture(cmdB, m_TransmittanceLutTex);
		}

		// Evaluate the multi scattering LUT
		{
			// Constant buffers
			d3d12::command_buffer::set_compute_shader_cbuffer(cmdB, m_MultiScatteringLutCS, "_SkyAtmosphereCB", m_SkyAtmosphereCB);

			// Input
			d3d12::command_buffer::set_compute_shader_texture(cmdB, m_MultiScatteringLutCS, "_TransmittanceLUTTexture", m_TransmittanceLutTex);

			// Output
			d3d12::command_buffer::set_compute_shader_texture(cmdB, m_MultiScatteringLutCS, "_MultiScatteringLUTTextureRW", m_MultiScatteringLutTex);

			// Samplers
			d3d12::command_buffer::set_compute_shader_sampler(cmdB, m_MultiScatteringLutCS, "_sampler_linear_clamp", m_LinearClampSampler);

			// Dispatch
			d3d12::command_buffer::dispatch(cmdB, m_MultiScatteringLutCS, multi_scat_lut_size, multi_scat_lut_size, 1);

			// Barrier
			d3d12::command_buffer::uav_barrier_texture(cmdB, m_MultiScatteringLutTex);
		}
	}
	d3d12::command_buffer::end_section(cmdB);
}

void Sky::update_constant_buffer(CommandBuffer cmdB)
{
	SkyAtmosphereCB skyCB;
	skyCB._AbsorptionExtinction = float3({ 0.000650f, 0.001881f, 0.000085f });
	skyCB._RayleighScattering = float3({ 0.005802f, 0.013558f, 0.033100f });
	skyCB._MisePhaseFunctionG = 0.8f;
	skyCB._MieScattering = float3({ 0.003996f, 0.003996f, 0.003996f });
	skyCB._PlanetRadiusSky = 6371000.0;
	skyCB._PlanetCenterSky = { 0.0f, 0.0f, 0.0f };
	skyCB._BottomRadius = 6371.0 - 1.0f;
	skyCB._MieExtinction = float3({ 0.004440f, 0.004440f, 0.004440f });
	skyCB._TopRadius = skyCB._BottomRadius * 1.013f;
	skyCB._MieAbsorption = max_zero(skyCB._MieExtinction - skyCB._MieScattering);
	skyCB._MiePhaseG = 0.8;
	skyCB._GroundAlbedo = float3({0.0, 0.01, 0.02});

	skyCB._RayleighDensity0LayerWidth = 0.0f;
	skyCB._RayleighDensity0ExpTerm = 0.0f;
	skyCB._RayleighDensity0ExpScale = 0.0f;
	skyCB._RayleighDensity0LinearTerm = 0.0f;
	skyCB._RayleighDensity0ConstantTerm = 0.0f;

	skyCB._RayleighDensity1LayerWidth = 0.0f;
	skyCB._RayleighDensity1ExpTerm = 1.0f;
	skyCB._RayleighDensity1ExpScale = -1.0f / 8.0f;
	skyCB._RayleighDensity1LinearTerm = 0.0f;
	skyCB._RayleighDensity1ConstantTerm = 0.0f;

	skyCB._MieDensity0LayerWidth = 0.0f;
	skyCB._MieDensity0ExpTerm = 0.0f;
	skyCB._MieDensity0ExpScale = 0.0f;
	skyCB._MieDensity0LinearTerm = 0.0f;
	skyCB._MieDensity0ConstantTerm = 0.0f;

	skyCB._MieDensity1LayerWidth = 0.0f;
	skyCB._MieDensity1ExpTerm = 1.0f;
	skyCB._MieDensity1ExpScale = -1.0f / 1.2f;
	skyCB._MieDensity1LinearTerm = 0.0f;
	skyCB._MieDensity1ConstantTerm = 0.0f;

	skyCB._AbsorptionDensity0LayerWidth = 25.0f;
	skyCB._AbsorptionDensity0ExpTerm = 0.0f;
	skyCB._AbsorptionDensity0ExpScale = 0.0f;
	skyCB._AbsorptionDensity0LinearTerm = 1.0f / 15.0f;
	skyCB._AbsorptionDensity0ConstantTerm = -2.0f / 3.0f;

	skyCB._AbsorptionDensity1LayerWidth = 0.0f;
	skyCB._AbsorptionDensity1ExpTerm = 0.0f;
	skyCB._AbsorptionDensity1ExpScale = 0.0f;
	skyCB._AbsorptionDensity1LinearTerm = -1.0f / 15.0f;
	skyCB._AbsorptionDensity1ConstantTerm = 8.0f / 3.0f;

	d3d12::resources::set_constant_buffer(m_SkyAtmosphereCB, (const char*)&skyCB, sizeof(SkyAtmosphereCB));
	d3d12::command_buffer::upload_constant_buffer(cmdB, m_SkyAtmosphereCB);
}

