#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"

#include <iostream>

namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle) :
			origin{ _origin },
			fovAngle{ _fovAngle }
		{
		}

		Vector3 origin{};
		float fovAngle{ 90.f };

		Vector3 forward{ Vector3::UnitZ };
		Vector3 up{ Vector3::UnitY };
		Vector3 right{ Vector3::UnitX };

		float totalPitch{ 0.f };
		float totalYaw{ 0.f };

		Matrix cameraToWorld{};

		Matrix CalculateCameraToWorld()
		{
			//todo: W2
			right = Vector3::Cross(Vector3::UnitY, forward);
			right.Normalize();

			up = Vector3::Cross(forward, right);
			up.Normalize();

			cameraToWorld[0] = Vector4{ right.x, right.y, right.z, 0 };
			cameraToWorld[1] = Vector4{ up.x, up.y, up.z, 0 };
			cameraToWorld[2] = Vector4{ forward.x, forward.y, forward.z, 0 };
			cameraToWorld[3] = Vector4{ origin.x, origin.y, origin.z, 1 };

			return cameraToWorld;
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();
			const float moveSpeed = 10.f;
			const float rotationSpeed = PI / 2.f;

			// Keyboard Input
			HandleKeyboardInput(deltaTime, moveSpeed);

			// Mouse Input
			HandleMouseInput(deltaTime, moveSpeed, rotationSpeed);

			// Rotate camera
			//Matrix finalRotation = Matrix::CreateRotation(totalPitch, totalYaw, 0);

			Matrix finalRotation = Matrix::CreateRotationX(totalPitch) * Matrix::CreateRotationY(totalYaw);

			forward = finalRotation.TransformVector(Vector3::UnitZ);
			//forward.Normalize();
		}

		void HandleKeyboardInput(float deltaTime, float moveSpeed)
		{
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);
			if (pKeyboardState[SDL_SCANCODE_W])
			{
				MoveForward(deltaTime, moveSpeed);
			}
			if (pKeyboardState[SDL_SCANCODE_S])
			{
				MoveBackward(deltaTime, moveSpeed);
			}
			if (pKeyboardState[SDL_SCANCODE_D])
			{
				origin.x += moveSpeed * deltaTime;
			}
			if (pKeyboardState[SDL_SCANCODE_A])
			{
				origin.x -= moveSpeed * deltaTime;
			}
			if (pKeyboardState[SDL_SCANCODE_Q])
			{
				MoveUp(deltaTime, moveSpeed);
			}
			if (pKeyboardState[SDL_SCANCODE_E])
			{
				MoveDown(deltaTime, moveSpeed);
			}
		}
		void HandleMouseInput(float deltaTime, float moveSpeed, float rotationSpeed)
		{
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);
			if (mouseState & SDL_BUTTON_LMASK && mouseState & SDL_BUTTON_RMASK)
			{
				if (mouseY < 0)
				{
					MoveUp(deltaTime, moveSpeed);
				}
				else if (mouseY > 0)
				{
					MoveDown(deltaTime, moveSpeed);
				}
			}
			else if (mouseState & SDL_BUTTON_RMASK)
			{
				if (mouseX != 0)
				{
					totalYaw += mouseX * rotationSpeed * deltaTime;
				}
				if (mouseY != 0)
				{
					totalPitch -= mouseY * rotationSpeed * deltaTime;
				}
			}
			//forward and backward translation with mouse
			else if (mouseState & SDL_BUTTON_LMASK)
			{
				if (mouseY < 0)
				{
					MoveForward(deltaTime, moveSpeed);
				}
				else if (mouseY > 0)
				{
					MoveBackward(deltaTime, moveSpeed);
				}
			}
		}

		void MoveForward(float deltaTime, float moveSpeed) { origin.z += deltaTime * moveSpeed; }
		void MoveBackward(float deltaTime, float moveSpeed) { origin.z -= deltaTime * moveSpeed; }
		void MoveUp(float deltaTime, float moveSpeed) { origin.y += deltaTime * moveSpeed; }
		void MoveDown(float deltaTime, float moveSpeed) { origin.y -= deltaTime * moveSpeed; }
	};
}
