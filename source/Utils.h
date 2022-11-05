#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"
#include <iostream>
#include <xmmintrin.h>	

namespace dae
{
	namespace Utils
	{
		inline float fastSqrt(float arg) {		
			return _mm_cvtss_f32(
				_mm_sqrt_ss(_mm_set_ps1(arg))
			);
		}

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

				if (isnan(normal.x))
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

			//float t0 = dp - sqrt(tcaSquared);
			float t0 = dp - Utils::fastSqrt(tcaSquared);

			if (t0 < ray.min || t0 > ray.max)
				return false;

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
			float dot2 = Vector3::Dot(ray.direction, plane.normal);
			if (dot2 > -FLT_EPSILON && dot2 < FLT_EPSILON)
				return false;

			float dot = Vector3::Dot((plane.origin - ray.origin), plane.normal);

			const float t = dot / dot2;

			if (t < ray.min || t > ray.max)
				return false;

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


			if (Vector3::Dot(triangle.normal, ray.direction) == 0)
				return false;

			Vector3 center = { (triangle.v0 + triangle.v1 + triangle.v2) / 3 };
			Vector3 L = center - ray.origin;
			float t = Vector3::Dot(L, triangle.normal) / Vector3::Dot(ray.direction, triangle.normal);

			if (t < ray.min || t > ray.max)
				return false;

			Vector3 p = ray.origin + t * ray.direction;
			
			Vector3 edgeA = triangle.v1 - triangle.v0;
			Vector3 pointToSide = p - triangle.v0;
			if (Vector3::Dot(triangle.normal, Vector3::Cross(edgeA, pointToSide)) < 0)
				return false;

			Vector3 edgeB = triangle.v2 - triangle.v1;
			pointToSide = p - triangle.v1;
			if (Vector3::Dot(triangle.normal, Vector3::Cross(edgeB, pointToSide)) < 0)
				return false;

			Vector3 edgeC = triangle.v0 - triangle.v2;
			pointToSide = p - triangle.v2;
			if (Vector3::Dot(triangle.normal, Vector3::Cross(edgeC, pointToSide)) < 0)
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

		inline bool HitTest_Triangle_MöllerTrumbore(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
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

			// Möller–Trumbore intersection algorithm. (2022, July 23). In Wikipedia.
			// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

			Vector3 edgeA = triangle.v1 - triangle.v0;
			Vector3 edgeB = triangle.v2 - triangle.v0;

			Vector3 h = Vector3::Cross(ray.direction, edgeB);
			float a = Vector3::Dot(edgeA, h);

			if (a < -FLT_EPSILON && a > FLT_EPSILON)
				return false; // This ray is parallel to this triangle

			const float f = 1.f / a;
			const Vector3 s = ray.origin - triangle.v0;
			const float u = f * Vector3::Dot(s, h);

			if (u < 0.f || u > 1.f)
				return false;

			const Vector3 q = Vector3::Cross(s, edgeA);
			const float v = f * Vector3::Dot(ray.direction, q);
			if (v < 0.f || u + v > 1.f)
				return false;

			// At this stage we can compute t to find out where the intersection point is on the line
			const float t = f * Vector3::Dot(edgeB, q);

			if (t > ray.max || t < ray.min)
				return false;

			if (!ignoreHitRecord)
			{
				hitRecord.didHit = true;
				hitRecord.origin = ray.origin + t * ray.direction;
				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.t = t;
				hitRecord.normal = triangle.normal;
			}

			return true;
		}
		inline bool HitTest_Triangle_MöllerTrumbore(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle_MöllerTrumbore(triangle, ray, temp, true);
		}

#pragma endregion
#pragma region TriangeMesh HitTest
		inline bool SlabTest_TriangleMesh(Vector3 minAABB, Vector3 maxAABB, const Ray& ray)
		{
			const float tx1 = (minAABB.x - ray.origin.x) / ray.direction.x;
			const float tx2 = (maxAABB.x - ray.origin.x) / ray.direction.x;

			float tmin = std::min(tx1, tx2);
			float tmax = std::max(tx1, tx2);

			const float ty1 = (minAABB.y - ray.origin.y) / ray.direction.y;
			const float ty2 = (maxAABB.y - ray.origin.y) / ray.direction.y;

			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));

			const float tz1 = (minAABB.z - ray.origin.z) / ray.direction.z;
			const float tz2 = (maxAABB.z - ray.origin.z) / ray.direction.z;

			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));

			return tmax > 0 && tmax >= tmin;
		}
#ifdef USE_SIMD
		float IntersectAABB_SSE(const Ray& ray, const __m128 bmin4, const __m128 bmax4)
		{
			static __m128 mask4 = _mm_cmpeq_ps(_mm_setzero_ps(), _mm_set_ps(1, 0, 0, 0));
			__m128 t1 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmin4, mask4), ray.O4), ray.rD4);
			__m128 t2 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmax4, mask4), ray.O4), ray.rD4);
			__m128 vmax4 = _mm_max_ps(t1, t2), vmin4 = _mm_min_ps(t1, t2);
			float tmax = std::min(vmax4.m128_f32[0], std::min(vmax4.m128_f32[1], vmax4.m128_f32[2]));
			float tmin = std::max(vmin4.m128_f32[0], std::max(vmin4.m128_f32[1], vmin4.m128_f32[2]));
		
			return tmax > 0 && tmax >= tmin;
		}
#endif // USE_SIMD

		inline void IntersectBVH(const TriangleMesh& mesh, const Ray& ray, const UINT nodeIdx,
			bool& hasHit, HitRecord& hitRecord, bool ignoreHitRecord,
			HitRecord& tempHitRecord, Triangle& triangle)
		{
			BVHNode& node = mesh.pBVHNodes[nodeIdx];

#ifdef USE_SIMD
			if (!IntersectAABB_SSE(ray, node.aabbMin4, node.aabbMax4)) return;
#else
			if (!SlabTest_TriangleMesh(node.aabbMin, node.aabbMax, ray)) return;
#endif // USE_SIMD
			if (node.primCount <= 0)
			{
				IntersectBVH(mesh, ray, node.leftChild, hasHit, hitRecord, ignoreHitRecord, tempHitRecord, triangle);
				IntersectBVH(mesh, ray, node.leftChild + 1, hasHit, hitRecord, ignoreHitRecord, tempHitRecord, triangle);
				return;
			}

			// For each triangle in the node
			for (UINT i{}; i < node.primCount; i += 3)
			{
				// Set the position and normal of the current triangle to the triangle object
				triangle.v0 = mesh.transformedPositions[mesh.indices[node.firstPrim + i]];
				triangle.v1 = mesh.transformedPositions[mesh.indices[node.firstPrim + i + 1]];
				triangle.v2 = mesh.transformedPositions[mesh.indices[node.firstPrim + i + 2]];
				triangle.normal = mesh.transformedNormals[(node.firstPrim + i) / 3];

				if (HitTest_Triangle_MöllerTrumbore(triangle, ray, tempHitRecord, ignoreHitRecord))
				{
					if (!ignoreHitRecord)
					{

						if (hitRecord.t > tempHitRecord.t)
							hitRecord = tempHitRecord;
					}
					hasHit = true;
				}
				else	
					continue;
			}
		}
		

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			bool hasHit{};

			HitRecord tempHitRecord{};

			Triangle triangle{};
			triangle.cullMode = mesh.cullMode;
			triangle.materialIndex = mesh.materialIndex;

#ifdef USE_BVH
			IntersectBVH(mesh, ray, 0, hasHit, hitRecord, ignoreHitRecord, tempHitRecord, triangle);
#else
			if (!SlabTest_TriangleMesh(mesh.transformedMinAABB, mesh.transformedMaxAABB, ray))
				return false;

			for (size_t i{}; i < mesh.indices.size(); i += 3)
			{
				triangle.v0 = mesh.transformedPositions[mesh.indices[i]];
				triangle.v1 = mesh.transformedPositions[mesh.indices[i + 1]];
				triangle.v2 = mesh.transformedPositions[mesh.indices[i + 2]];

				triangle.normal = mesh.transformedNormals[i / 3];
			
				if (HitTest_Triangle_MöllerTrumbore(triangle, ray, tempHitRecord, ignoreHitRecord))
				{
					if (!ignoreHitRecord)
					{
						if (hitRecord.t > tempHitRecord.t)
							hitRecord = tempHitRecord;
					}
					hasHit = true;
				}
			}
#endif // USE_BVH

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
			ColorRGB E = colors::White;

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
}