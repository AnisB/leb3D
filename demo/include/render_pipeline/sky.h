#pragma once

// Internal includes
#include "graphics/types.h"
#include "render_pipeline/camera.h"

// External includes
#include <string>

class Sky
{
public:
	Sky();
	~Sky();

	// Init and Release functions
	void initialize(GraphicsDevice device);
	void release();

	// Reload the shaders
	void reload_shaders(const std::string& shaderLibrary);

	// Pre-rendering steps
	void pre_render(CommandBuffer cmd);

	// Access the resources
	Texture transmittance_lut() const { return m_TransmittanceLutTex; }
	Texture multi_scattering_lut() const { return m_MultiScatteringLutTex; }
	ConstantBuffer constant_buffer() const { return m_SkyAtmosphereCB; }

private:
	void update_constant_buffer(CommandBuffer cmdB);

private:
	// Generic graphics resources
	GraphicsDevice m_Device = 0;

	// Buffers that hold the precomputations
	Texture m_TransmittanceLutTex = 0;
	Texture m_MultiScatteringLutTex = 0;
	Texture m_SkyViewLutTex = 0;

	// Required shaders
	ComputeShader m_TransmittanceLutCS = 0;
	ComputeShader m_MultiScatteringLutCS = 0;

	// Sky constant buffer
	ConstantBuffer m_SkyAtmosphereCB = 0;

	// Sampler
	Sampler m_LinearClampSampler = 0;
};