//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"

#include <future>	// async
#include <ppl.h>	// parallel_for

using namespace dae;

//#define ASYNC 
#define PARALLEL_FOR

Renderer::Renderer(SDL_Window* pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow)),
	m_CurrentLightingMode(LightingMode::Combined),
	m_ShadowsEnabled(true)
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	camera.CalculateCameraToWorld();

	float aspectRatio{ m_Width / float(m_Height) };
	float FOV{ tanf((camera.fovAngle * TO_RADIANS) / 2.f) };

	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	const uint32_t numPixels = m_Width * m_Height;

#if defined(ASYNC)
	//Async execution
	const uint32_t numCores = std::thread::hardware_concurrency();
	std::vector<std::future<void>> async_futures{};
	const uint32_t numPixelsPerTask = numPixels / numCores;
	uint32_t numUnassignedPixels = numPixels % numCores;
	uint32_t currPixelIndex = 0;

	for (uint32_t coreId{ 0 }; coreId < numCores; ++coreId)
	{
		uint32_t taskSize = numPixelsPerTask;
		if (numUnassignedPixels > 0)
		{
			++taskSize;
			--numUnassignedPixels;
		}

		async_futures.push_back(
			std::async(std::launch::async, [=, this]
				{
					const uint32_t pixelIndexEnd = currPixelIndex + taskSize;
					for (uint32_t pixelIndex{ currPixelIndex }; pixelIndex < pixelIndexEnd; ++pixelIndex)
					{
						RenderPerPixel(pScene, pixelIndex, FOV, aspectRatio, camera, lights, materials);
					}
				}
			)
		);

		currPixelIndex += taskSize;
	}

	// wait till all tasks are finished
	for (const std::future<void>& f : async_futures)
	{
		f.wait();
	}
#elif defined(PARALLEL_FOR)
	// Parallel for execution
	concurrency::parallel_for(0u, numPixels, [=, this](int i)
		{
			RenderPixel(pScene, i, FOV, aspectRatio, camera, lights, materials);
		});
#else 
	// Synchronous Execution
	for (uint32_t i{0}; i < numPixels; ++i)
	{
		RenderPerPixel(pScene, i, FOV, aspectRatio, camera, lights, materials);
	}
#endif

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

void dae::Renderer::RenderPixel(Scene* pScene, uint32_t pixelIndex, float fov, float aspectRatio, const Camera& camera, const std::vector<Light>& lights, const std::vector<Material*>& materials) const
{
	const int px = pixelIndex % m_Width;
	const int py = pixelIndex / m_Width;

	const float rx = px + 0.5f;
	const float ry = py + 0.5f;

	const float cx = (2 * ((px) / float(m_Width)) - 1) * aspectRatio * fov;
	const float cy = (1 - 2 * ((py) / float(m_Height))) * fov;

	Vector3 rayDirection{};
	rayDirection.x = cx;
	rayDirection.y = cy;
	rayDirection.z = 1;

	rayDirection = camera.cameraToWorld.TransformVector(rayDirection);
	rayDirection.Normalize();
		
	// For each pixel ...
	// ... Ray Direction calculations above
	// Ray we are casting from the camera towards each pixel
	Ray viewRay{ camera.origin, rayDirection };

	// Color to write to the color buffer (default = black)
	ColorRGB finalColor = {};

	// HitRecord containing more information about a potential hit
	HitRecord closestHit{};
	pScene->GetClosestHit(viewRay, closestHit);

	if (closestHit.didHit)
	{
		// If we hit something, set finalColor to material color, else keep black
		// Use HitRecord::materialIndex to find the corresponding material
		for (const Light& light : lights)
		{
			ColorRGB tempCycleColor = colors::White;

			const float offSet{ 0.001f };
			closestHit.origin += closestHit.normal * offSet;

			Vector3 directionToLight = LightUtils::GetDirectionToLight(light, closestHit.origin);
			
			if (m_ShadowsEnabled)
			{
				const float lightRayLength = directionToLight.Normalize();

				Ray lightRay{ closestHit.origin, directionToLight };
				lightRay.min = offSet;
				lightRay.max = lightRayLength;

				if (pScene->DoesHit(lightRay))
					continue;
			}

			directionToLight = directionToLight.Normalized();
			
			float lambertCos = Vector3::Dot(closestHit.normal, directionToLight);
			if (lambertCos < 0)
				continue;

			switch (m_CurrentLightingMode)
			{
			case LightingMode::ObservedArea:
			{
				tempCycleColor *= ColorRGB{ lambertCos,lambertCos,lambertCos };
			}
			break;
			case LightingMode::Radiance:
				tempCycleColor *= LightUtils::GetRadiance(light, closestHit.origin);
				break;
			case LightingMode::BRDF:
				tempCycleColor *= materials[closestHit.materialIndex]->Shade(closestHit, directionToLight, -viewRay.direction);
				break;
			case LightingMode::Combined:
			{
				tempCycleColor *= LightUtils::GetRadiance(light, closestHit.origin) * lambertCos
					* materials[closestHit.materialIndex]->Shade(closestHit, directionToLight, -viewRay.direction);
				break;
			}
			}	

			finalColor += tempCycleColor;
		}
	}

	// Update Color in Buffer
	finalColor.MaxToOne();

	m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode() { m_CurrentLightingMode = LightingMode{ ((int)m_CurrentLightingMode + 1) % 4 }; }
void dae::Renderer::ToggleShadows() { m_ShadowsEnabled = !m_ShadowsEnabled; }
