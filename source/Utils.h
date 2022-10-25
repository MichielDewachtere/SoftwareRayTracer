#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"
#include <iostream>

namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//todo W1
			float dp = Vector3::Dot(sphere.origin - ray.origin, ray.direction);
			float tclSquared = (sphere.origin - ray.origin).SqrMagnitude();
			
			float odSquared = tclSquared - (dp * dp);
			if (odSquared > sphere.radius * sphere.radius)
				return false;

			float tcaSquared = sphere.radius * sphere.radius - odSquared;
			float t0 = dp - sqrt(tcaSquared);

			if (t0 < ray.max && t0 > ray.min)
			{
				if (!ignoreHitRecord)
				{
					hitRecord.origin = ray.origin + ray.direction * t0;
					hitRecord.normal = Vector3{ hitRecord.origin - sphere.origin } / sphere.radius;
					hitRecord.t = t0;
					hitRecord.didHit = true;
					hitRecord.materialIndex = sphere.materialIndex;
				}
				return true;
			}

			return false;
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//todo W1 COMPLETED
			const float t = Vector3::Dot((plane.origin - ray.origin), plane.normal) / Vector3::Dot(ray.direction, plane.normal);
			if (t > ray.min && t < ray.max)
			{
				if (!ignoreHitRecord)
				{
					hitRecord.didHit = true;
					hitRecord.origin = ray.origin + ray.direction * t;
					hitRecord.materialIndex = plane.materialIndex;
					hitRecord.t = t;
					hitRecord.normal = plane.normal;
				}
				return true;
			}
			return false;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//todo W5
			TriangleCullMode cullingMode = triangle.cullMode;

			if (!ignoreHitRecord)
			{
				switch (cullingMode)
				{
				case dae::TriangleCullMode::FrontFaceCulling:
					cullingMode = TriangleCullMode::BackFaceCulling;
					break;
				case dae::TriangleCullMode::BackFaceCulling:
					cullingMode = TriangleCullMode::FrontFaceCulling;
					break;
				}
			}

			switch (cullingMode)
			{
			case TriangleCullMode::FrontFaceCulling:
				if (Vector3::Dot(triangle.normal, ray.direction) > 0)
					return false;
				break;
			case TriangleCullMode::BackFaceCulling:
				if (Vector3::Dot(triangle.normal, ray.direction) < 0)
					return false;
				break;
			}

			Vector3 edgeA = triangle.v1 - triangle.v0;
			Vector3 edgeB = triangle.v2 - triangle.v1;
			Vector3 edgeC = triangle.v0 - triangle.v2;

			if (Vector3::Dot(triangle.normal, ray.direction) == 0)
				return false;

			Vector3 center = { (triangle.v0 + triangle.v1 + triangle.v2) / 3 };
			Vector3 L = center - ray.origin;
			float t = Vector3::Dot(L, triangle.normal) / Vector3::Dot(ray.direction, triangle.normal);

			if (t < ray.min || t > ray.max)
				return false;

			Vector3 p = ray.origin + t * ray.direction;
			
			Vector3 pointToSideA = p - triangle.v0;
			if (Vector3::Dot(triangle.normal, Vector3::Cross(edgeA, pointToSideA)) < 0)
				return false;

			Vector3 pointToSideB = p - triangle.v1;
			if (Vector3::Dot(triangle.normal, Vector3::Cross(edgeB, pointToSideB)) < 0)
				return false;

			Vector3 pointToSideC = p - triangle.v2;
			if (Vector3::Dot(triangle.normal, Vector3::Cross(edgeC, pointToSideC)) < 0)
				return false;

			if (!ignoreHitRecord)
			{
				hitRecord.didHit = true;
				hitRecord.origin = p;
				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.t = t;
				hitRecord.normal = triangle.normal;
			}

			return true;
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			//todo W5
			HitRecord tempHitRecord{};
			bool hasHit{};

			Triangle triangle{};
			triangle.cullMode = mesh.cullMode;
			triangle.materialIndex = mesh.materialIndex;

			for (size_t i{}; i < mesh.indices.size(); i += 3)
			{
				triangle.v0 = mesh.transformedPositions[mesh.indices[i]];
				triangle.v1 = mesh.transformedPositions[mesh.indices[i + 1]];
				triangle.v2 = mesh.transformedPositions[mesh.indices[i + 2]];

				triangle.normal = mesh.transformedNormals[i / 3];
				
				if (HitTest_Triangle(triangle, ray, tempHitRecord, ignoreHitRecord))
				{
					if (!ignoreHitRecord)
					{
						if (hitRecord.t > tempHitRecord.t)
							hitRecord = tempHitRecord;
					}
					hasHit = true;
				}
			}
			return hasHit;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}
#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			//todo W3
			return Vector3{ light.origin - origin };
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			//todo W3
			ColorRGB E{ 1,1,1 };

			switch (light.type)
			{
			case LightType::Point:
			{
				float distance{ (light.origin - target).SqrMagnitude() };
				E = light.color * (light.intensity / distance);
				break;
			}
			case LightType::Directional:
			{
				E = light.color * light.intensity;
				break;
			}
			}

			return E;
		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof()) 
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if(isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}
}