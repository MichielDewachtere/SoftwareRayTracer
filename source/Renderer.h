#pragma once

#include <cstdint>

#include <vector>

struct SDL_Window;
struct SDL_Surface;

namespace dae
{
	class Scene;
	class Material;
	struct Camera;
	struct Light;

	class Renderer final
	{
	public:
		Renderer(SDL_Window* pWindow);
		~Renderer() = default;

		Renderer(const Renderer&) = delete;
		Renderer(Renderer&&) noexcept = delete;
		Renderer& operator=(const Renderer&) = delete;
		Renderer& operator=(Renderer&&) noexcept = delete;

		void Render(Scene* pScene) const;

		void RenderPixel(Scene* pScene, 
			uint32_t pixelIndex, 
			float fov, 
			float aspectRatio, 
			const Camera& camera, 
			const std::vector<Light>& lights, 
			const std::vector<Material*>& materials) const;

		bool SaveBufferToImage() const;

		void CycleLightingMode();
		void ToggleShadows();
	private:
		enum class LightingMode 
		{
			ObservedArea, //Lambert Cosine Law
			Radiance, // Incident Radiance
			BRDF, // Scattering of the light
			Combined // ObservedArea * Radiance * BRDF
		};

		LightingMode m_CurrentLightingMode{};
		bool m_ShadowsEnabled{};

		SDL_Window* m_pWindow{};

		SDL_Surface* m_pBuffer{};
		uint32_t* m_pBufferPixels{};

		int m_Width{};
		int m_Height{};
	};
}
