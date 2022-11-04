#pragma once
#include <cassert>

#include "Math.h"
#include "vector"

#define UINT unsigned int
//#define USE_BVH

namespace dae
{
#pragma region GEOMETRY
	struct Sphere
	{
		Vector3 origin{};
		float radius{};

		unsigned char materialIndex{ 0 };
	};

	struct Plane
	{
		Vector3 origin{};
		Vector3 normal{};

		unsigned char materialIndex{ 0 };
	};

	enum class TriangleCullMode
	{
		FrontFaceCulling,
		BackFaceCulling,
		NoCulling
	};

	// How to build a BVH. (2022, April 13). From JBikker
	// https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/

	struct BVHNode
	{
		Vector3 aabbMin, aabbMax;	// 24 bytes
		UINT leftChild;				// 4 bytes
		UINT firstPrim;				// 4 bytes
		UINT primCount;				// 4 bytes; total: 36 bytes
	};

	struct Triangle
	{
		Triangle() = default;
		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2, const Vector3& _normal) :
			v0{ _v0 }, v1{ _v1 }, v2{ _v2 }, normal{ _normal.Normalized() } {}

		Triangle(const Vector3& _v0, const Vector3& _v1, const Vector3& _v2) :
			v0{ _v0 }, v1{ _v1 }, v2{ _v2 }
		{
			const Vector3 edgeV0V1 = v1 - v0;
			const Vector3 edgeV0V2 = v2 - v0;
			normal = Vector3::Cross(edgeV0V1, edgeV0V2).Normalized();
		}

		Vector3 v0{};
		Vector3 v1{};
		Vector3 v2{};

		Vector3 normal{};

		TriangleCullMode cullMode{};
		unsigned char materialIndex{};
	};

	struct TriangleMesh
	{
		TriangleMesh() = default;
		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, TriangleCullMode _cullMode):
		positions(_positions), indices(_indices), cullMode(_cullMode)
		{
			//Calculate Normals
			CalculateNormals();

			//Update Transforms
			UpdateTransforms();
		}

		TriangleMesh(const std::vector<Vector3>& _positions, const std::vector<int>& _indices, const std::vector<Vector3>& _normals, TriangleCullMode _cullMode) :
			positions(_positions), indices(_indices), normals(_normals), cullMode(_cullMode)
		{
			UpdateTransforms();
		}

		~TriangleMesh()
		{
			delete[] pBVHNodes;
		}

		std::vector<Vector3> positions{};
		std::vector<Vector3> normals{};
		std::vector<int> indices{};
		unsigned char materialIndex{};

		TriangleCullMode cullMode{TriangleCullMode::BackFaceCulling};

		Matrix rotationTransform{};
		Matrix translationTransform{};
		Matrix scaleTransform{};

		Vector3 minAABB;
		Vector3 maxAABB;

		Vector3 transformedMinAABB;
		Vector3 transformedMaxAABB;

		std::vector<Vector3> transformedPositions{};
		std::vector<Vector3> transformedNormals{};

		BVHNode* pBVHNodes{};
		UINT rootNodeIdx{};
		UINT nodesUsed{};

		void Translate(const Vector3& translation)
		{
			translationTransform = Matrix::CreateTranslation(translation);
		}

		void RotateY(float yaw)
		{
			rotationTransform = Matrix::CreateRotationY(yaw);
		}

		void Scale(const Vector3& scale)
		{
			scaleTransform = Matrix::CreateScale(scale);
		}

		void AppendTriangle(const Triangle& triangle, bool ignoreTransformUpdate = false)
		{
			int startIndex = static_cast<int>(positions.size());

			positions.push_back(triangle.v0);
			positions.push_back(triangle.v1);
			positions.push_back(triangle.v2);

			indices.push_back(startIndex);
			indices.push_back(++startIndex);
			indices.push_back(++startIndex);

			normals.push_back(triangle.normal);

			//Not ideal, but making sure all vertices are updated
			if(!ignoreTransformUpdate)
				UpdateTransforms();
		}

		void CalculateNormals()
		{
			for (size_t i{}; i < indices.size(); i += 3)
			{
				const Vector3 edgeV0V1 = positions[indices[i + 1]] - positions[indices[i]];
				const Vector3 edgeV0V2 = positions[indices[i + 2]] - positions[indices[i]];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2).Normalized();

				normals.emplace_back(normal);
			}
		}

		void UpdateTransforms()
		{
			transformedNormals.clear();
			transformedNormals.reserve(normals.size());
			transformedPositions.clear();
			transformedPositions.reserve(positions.size());

			//Calculate Final Transform 
			const auto finalTransform = scaleTransform * rotationTransform * translationTransform;
			
			//Transform Positions (positions > transformedPositions)
			for (auto& position : positions)
				transformedPositions.emplace_back(finalTransform.TransformPoint(position));
			
			//Update AABB
			UpdateTransformedAABB(finalTransform);

			//Transform Normals (normals > transformedNormals)
			for (auto& normal : normals)
				transformedNormals.emplace_back(finalTransform.TransformVector(normal));
		}

		void UpdateAABB()
		{
			if (positions.size() > 0)
			{
				minAABB = positions[0];
				maxAABB = positions[0];

				for (auto& p : positions)
				{
					minAABB = Vector3::Min(p, minAABB);
					maxAABB = Vector3::Max(p, maxAABB);
				}
			}
		}

		void UpdateTransformedAABB(const Matrix& finalTransform)
		{
			// AABB update: be careful -> transform the 8 vertices of the aabb
			// and alculate new min and max
			
			//    H--------G
			//   /|       /|
			//  / |      / |
			// E--------F  |
			// |  |     |  |
			// |  D-----|--C
			// | /      | /
			// |/       |/
			// A--------B

			// (xmin, ymin, zmin) A
			Vector3 tMinAABB = finalTransform.TransformPoint(minAABB);
			Vector3 tMaxAABB = tMinAABB;
			// (xmax, ymin, zmin) B
			Vector3 tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmax, ymin, zmax) C
			tAABB = finalTransform.TransformPoint(maxAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmin, ymin, zmax) D
			tAABB = finalTransform.TransformPoint(minAABB.x, minAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmin, ymax, zmin) E
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmax, ymax, zmin) F
			tAABB = finalTransform.TransformPoint(maxAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmax, ymax, zmax) G
			tAABB = finalTransform.TransformPoint(maxAABB.x, maxAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmin, ymax, zmin) H
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, minAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

			transformedMinAABB = tMinAABB;
			transformedMaxAABB = tMaxAABB;
		}

// How to build a BVH. (2022, April 13). From JBikker
// https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/
#pragma region BVH
		void UpdateBVH()
		{
			nodesUsed = 0;

			BVHNode& root = pBVHNodes[rootNodeIdx];
			root.leftChild = 0;
			root.firstPrim = 0;
			root.primCount = static_cast<UINT>(indices.size());

			UpdateNodeBounds(rootNodeIdx);

			Subdivide(rootNodeIdx);
		}

		void UpdateNodeBounds(UINT nodeIdx)
		{
			BVHNode& node = pBVHNodes[nodeIdx];
			node.aabbMin = Vector3({ FLT_MAX, FLT_MAX, FLT_MAX });
			node.aabbMax = Vector3({ FLT_MIN, FLT_MIN, FLT_MIN });

			for (UINT i{ node.firstPrim }; i < node.firstPrim + node.primCount; ++i)
			{
				Vector3& vertex{ transformedPositions[indices[i]] };

				node.aabbMin = Vector3::Min(node.aabbMin, vertex);
				node.aabbMax = Vector3::Max(node.aabbMax, vertex);
			}
		}

		void Subdivide(UINT nodeIdx)
		{
			// terminate recursion
			BVHNode& node = pBVHNodes[nodeIdx];
			if (node.primCount <= 6) return;

			// determine split axis and position
			Vector3 extent = node.aabbMax - node.aabbMin;
			int axis = 0;
			if (extent.y > extent.x) axis = 1;
			if (extent.z > extent[axis]) axis = 2;
			float splitPos = node.aabbMin[axis] + extent[axis] * 0.5f;

			// in-place partition
			int i = static_cast<int>(node.firstPrim);
			int j = i + static_cast<int>(node.primCount) - 1;
			while (i <= j)
			{
				Vector3 centroid = { 
					(transformedPositions[indices[i]] 
					+ transformedPositions[indices[i + 1]] 
					+ transformedPositions[indices[i + 2]]) 
					/ 3.0f };
			

				if (centroid[axis] < splitPos)
					i += 3;
				else
				{
					std::swap(indices[i], indices[j - 2]);
					std::swap(indices[i + 1], indices[j - 1]);
					std::swap(indices[i+2], indices[j]);

					std::swap(normals[i / 3], normals[(j - 2) / 3]);
					std::swap(transformedNormals[i / 3], transformedNormals[(j - 2) / 3]);

					j -= 3;
				}
			}

			// abort split if one of the sides is empty
			UINT leftCount = i - node.firstPrim;
			if (leftCount == 0 || leftCount == node.primCount) 
				return;

			// create child nodes
			UINT leftChildIdx = ++nodesUsed;
			UINT rightChildIdx = ++nodesUsed;

			node.leftChild = leftChildIdx;

			pBVHNodes[leftChildIdx].firstPrim = node.firstPrim;
			pBVHNodes[leftChildIdx].primCount = leftCount;
			pBVHNodes[rightChildIdx].firstPrim = i;
			pBVHNodes[rightChildIdx].primCount = node.primCount - leftCount;
			node.primCount = 0;

			UpdateNodeBounds(leftChildIdx);
			UpdateNodeBounds(rightChildIdx);

			// recurse
			Subdivide(leftChildIdx);
			Subdivide(rightChildIdx);
		}
#pragma endregion
	};
#pragma endregion
#pragma region LIGHT
	enum class LightType
	{
		Point,
		Directional
	};

	struct Light
	{
		Vector3 origin{};
		Vector3 direction{};
		ColorRGB color{};
		float intensity{};

		LightType type{};
	};
#pragma endregion
#pragma region MISC
	struct Ray
	{
		Vector3 origin{};
		Vector3 direction{};

		float min{ 0.0001f };
		float max{ FLT_MAX };
	};

	struct HitRecord
	{
		Vector3 origin{};
		Vector3 normal{};
		float t = FLT_MAX;

		bool didHit{ false };
		unsigned char materialIndex{ 0 };
	};
#pragma endregion
}