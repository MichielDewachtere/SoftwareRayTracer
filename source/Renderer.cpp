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

using namespace dae;

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
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	float aspectRatio{ m_Width / float(m_Height) };
	float FOV{ tanf((camera.fovAngle * TO_RADIANS) / 2.f) };

	const Matrix cameraToWorld = camera.CalculateCameraToWorld();

	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{
			Vector3 rayDirection{};
			rayDirection.x = (2 * (px + 0.5f) / float(m_Width) - 1) * aspectRatio * FOV;
			rayDirection.y = (1 - 2 * (py + 0.5f) / float(m_Height)) * FOV;
			rayDirection.z = 1;

			rayDirection = cameraToWorld.TransformVector(rayDirection);

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
					ColorRGB tempCycleColor{ 1,1,1 };

					const float offSet{ 0.01f };
					closestHit.origin += closestHit.normal * offSet;

					Vector3 directionToLight = LightUtils::GetDirectionToLight(light, closestHit.origin);
					auto lightRayLength = directionToLight.Normalize();
					directionToLight = directionToLight.Normalized();

					Ray lightRay{ closestHit.origin, directionToLight };
					lightRay.min = offSet;
					lightRay.max = lightRayLength;

					if (pScene->DoesHit(lightRay) && m_ShadowsEnabled)
						continue;
					
					float lambertCos = Vector3::Dot(closestHit.normal, directionToLight);

					switch (m_CurrentLightingMode)
					{
					case LightingMode::ObservedArea:
					{
						if (lambertCos < 0)
							continue;
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
						if (lambertCos < 0)
							continue;
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
	}

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void dae::Renderer::CycleLightingMode() { m_CurrentLightingMode = LightingMode{ ((int)m_CurrentLightingMode + 1) % 4 }; }
void dae::Renderer::ToggleShadows() { m_ShadowsEnabled = !m_ShadowsEnabled; }
