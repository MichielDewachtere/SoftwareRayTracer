#pragma once
#include <cassert>

#include <xmmintrin.h>

#include "Math.h"
#include "vector"

//#define USE_BVH

#ifdef USE_BVH
	#define Part1_Basics
	//#define Part2_Fast_Rays
	//#define Part3_Quick_Build
#endif // USE_BVH

//#define USE_SIMD

#define UINT unsigned int
#define BINS 8

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

	// How to build a BVH - Basics. (2022, April 13). From JBikker
	// https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/
	struct BVHNode
	{
#ifdef USE_SIMD
		union
		{
			struct { Vector3 aabbMin; UINT leftFirst; };
			__m128 aabbMin4;
		};
		union
		{
			struct { Vector3 aabbMax; UINT primCount; };
			__m128 aabbMax4;
		};
#else
		Vector3 aabbMin, aabbMax;	// 24 bytes
		UINT leftChild;				// 4 bytes
		UINT firstPrim;				// 4 bytes
		UINT primCount;				// 4 bytes; total: 36 bytes
#endif // USE_SIMD
	};

	// How to build a BVH - Faster Rays. (2022, April 18). From JBikker
	// https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/
	struct AABB
	{
		Vector3 bmin = Vector3{ FLT_MAX, FLT_MAX, FLT_MAX };
		Vector3 bmax = Vector3{ FLT_MIN, FLT_MIN, FLT_MIN };
		
		void Grow(Vector3 p)
		{
			bmin = Vector3::Min(bmin, p);
			bmax = Vector3::Max(bmax, p);
		}

		void Grow(AABB& p)
		{
			bmin = Vector3::Min(bmin, p.bmin);
			bmax = Vector3::Max(bmax, p.bmax);
		}

		float Area()
		{
			Vector3 e = bmax - bmin; // box extent
			return e.x * e.y + e.y * e.z + e.z * e.x;
		}
	};

	// How to build a BVH - Quick Builds. (2022, April 21). From JBikker
	// https://jacco.ompf2.com/2022/04/21/how-to-build-a-bvh-part-3-quick-builds/
	struct Bin
	{
		AABB bounds; 
		int primCount = 0;
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

			//Not ideal, but making sure all vertices's are updated
			if(!ignoreTransformUpdate)
				UpdateTransforms();
		}

		void CalculateNormals()
		{
			normals.resize(indices.size() / 3);

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
			for (const auto& position : positions)
				transformedPositions.emplace_back(finalTransform.TransformPoint(position));
			
			//Transform Normals (normals > transformedNormals)
			for (const auto& normal : normals)
				transformedNormals.emplace_back(finalTransform.TransformVector(normal));

			//Update AABB
			UpdateTransformedAABB(finalTransform);
		}

		void UpdateAABB()
		{
			if (positions.size() > 0)
			{
				minAABB = positions[0];
				maxAABB = positions[0];

				for (const auto& p : positions)
				{
					minAABB = Vector3::Min(p, minAABB);
					maxAABB = Vector3::Max(p, maxAABB);
				}
			}
		}

		void UpdateTransformedAABB(const Matrix& finalTransform)
		{
			// AABB update: be careful -> transform the 8 vertices of the aabb
			// and calculate new min and max
			
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
			tAABB = finalTransform.TransformPoint(maxAABB);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);
			// (xmin, ymax, zmax) H
			tAABB = finalTransform.TransformPoint(minAABB.x, maxAABB.y, maxAABB.z);
			tMinAABB = Vector3::Min(tAABB, tMinAABB);
			tMaxAABB = Vector3::Max(tAABB, tMaxAABB);

			transformedMinAABB = tMinAABB;
			transformedMaxAABB = tMaxAABB;
		}

// How to build a BVH - Basics. (2022, April 13). From JBikker
// https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/
#pragma region BVH_Basics
		void UpdateBVH()
		{
			nodesUsed = 0;

			BVHNode& root = pBVHNodes[rootNodeIdx];
			root.firstPrim = 0;
			root.leftChild = 0;
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

#ifdef Part1_Basics
			Vector3 extent = node.aabbMax - node.aabbMin;
			int axis = 0;
			if (extent.y > extent.x) axis = 1;
			if (extent.z > extent[axis]) axis = 2;
			float splitPos = node.aabbMin[axis] + extent[axis] * 0.5f;
#elif defined(Part2_Fast_Rays)
			// How to build a BVH - Faster Rays. (2022, April 18). From JBikker
			// https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/
			// determine split axis using SAH
			int bestAxis = -1;
			float bestPos = 0, bestCost = FLT_MAX;
			for (int axis = 0; axis < 3; axis++) for (UINT i = 0; i < node.primCount; i++)
			{
				const Vector3 centroid = {
					(transformedPositions[indices[i]]
					+ transformedPositions[indices[i + 1]]
					+ transformedPositions[indices[i + 2]])
					/ 3.0f };
				const float candidatePos = centroid[axis];
				const float cost = EvaluateSAH(node, axis, candidatePos);
				if (cost < bestCost)
					bestPos = candidatePos, bestAxis = axis, bestCost = cost;
			}
			const int axis = bestAxis;
			const float splitPos = bestPos;

			const Vector3 e = node.aabbMax - node.aabbMin; // extent of parent
			const float parentArea = e.x * e.y + e.y * e.z + e.z * e.x;
			const float parentCost = node.primCount * parentArea;

			if (bestCost >= parentCost) return;
#elif defined(Part3_Quick_Build)
			// How to build a BVH - Quick Builds. (2022, April 21). From JBikker
			// https://jacco.ompf2.com/2022/04/21/how-to-build-a-bvh-part-3-quick-builds/
			// determine split axis using SAH
			int axis{ -1 };
			float splitPos{ 0 };

			const float splitCost{ FindBestSplitPlane(node, axis, splitPos) };

			const float noSplitCost{ CalculateNodeCost(node) };
			if (splitCost >= noSplitCost) return;
#else
			float splitPos{}, axis{};
#endif // Part1

			// in-place partition
			int i = static_cast<int>(node.firstPrim);
			int j = i + static_cast<int>(node.primCount) - 1;
			while (i <= j)
			{
				const Vector3 centroid = { 
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
			const UINT leftCount = i - node.firstPrim;
			if (leftCount == 0 || leftCount == node.primCount) 
				return;

			// create child nodes
			const UINT leftChildIdx = ++nodesUsed;
			const UINT rightChildIdx = ++nodesUsed;

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
// How to build a BVH - Faster Rays. (2022, April 18). From JBikker
// https://jacco.ompf2.com/2022/04/18/how-to-build-a-bvh-part-2-faster-rays/
#pragma region Faster_Rays
		float EvaluateSAH(BVHNode& node, int axis, float pos) const
		{
			// determine triangle counts and bounds for this split candidate
			AABB leftBox{}, rightBox{};
			int leftCount = 0, rightCount = 0;
			for (UINT i = 0; i < node.primCount; i += 3)
			{
				const Vector3 vertex0 = transformedPositions[indices[i]];
				const Vector3 vertex1 = transformedPositions[indices[i + 1]];
				const Vector3 vertex2 = transformedPositions[indices[i + 2]];
				
				const Vector3 centroid = { (vertex0 + vertex1 + vertex2) / 3.0f };

				if (centroid[axis] < pos)
				{
					++leftCount;
					leftBox.Grow(vertex0);
					leftBox.Grow(vertex1);
					leftBox.Grow(vertex2);
				}
				else
				{
					++rightCount;
					rightBox.Grow(vertex0);
					rightBox.Grow(vertex1);
					rightBox.Grow(vertex2);
				}
			}
			const float cost = leftCount * leftBox.Area() + rightCount * rightBox.Area();
			return cost > 0 ? cost : FLT_MAX;
		}
#pragma endregion
// How to build a BVH - Quick Builds. (2022, April 21). From JBikker
// https://jacco.ompf2.com/2022/04/21/how-to-build-a-bvh-part-3-quick-builds/
#pragma region Quick_Builds
		float FindBestSplitPlane(BVHNode& node, int& axis, float& splitPos) const
		{
			float bestCost = FLT_MAX;
			for (int a = 0; a < 3; a++)
			{
				float boundsMin = FLT_MAX, boundsMax = FLT_MIN;
				for (UINT i = 0; i < node.primCount; i += 3)
				{
					const Vector3 vertex0 = transformedPositions[indices[i]];
					const Vector3 vertex1 = transformedPositions[indices[i + 1]];
					const Vector3 vertex2 = transformedPositions[indices[i + 2]];

					const Vector3 centroid = { (vertex0 + vertex1 + vertex2) / 3.0f };

					boundsMin = std::min(boundsMin, centroid[a]);
					boundsMax = std::max(boundsMax, centroid[a]);
				}

				//if (boundsMin == boundsMax) continue;
				if (abs(boundsMin - boundsMax) < FLT_EPSILON) continue;
				
				// populate the bins
				Bin bin[BINS];
				float scale = BINS / (boundsMax - boundsMin);
				for (UINT i = 0; i < node.primCount; i += 3)
				{
					const Vector3 vertex0 = transformedPositions[indices[i]];
					const Vector3 vertex1 = transformedPositions[indices[i + 1]];
					const Vector3 vertex2 = transformedPositions[indices[i + 2]];

					const Vector3 centroid = { (vertex0 + vertex1 + vertex2) / 3.0f };

					const int binIdx = std::min(BINS - 1, (int)((centroid[a] - boundsMin) * scale));
					
					bin[binIdx].primCount += 3;
					bin[binIdx].bounds.Grow(vertex0);
					bin[binIdx].bounds.Grow(vertex1);
					bin[binIdx].bounds.Grow(vertex2);
				}
				
				// gather data for the 7 planes between the 8 bins
				float leftArea[BINS - 1]{};
				float rightArea[BINS - 1]{};

				int leftCount[BINS - 1]{};
				int rightCount[BINS - 1]{};

				AABB leftBox{};
				AABB rightBox{};

				int leftSum = 0;
				int rightSum = 0;

				for (int i = 0; i < BINS - 1; i++)
				{
					leftSum += bin[i].primCount;
					leftCount[i] = leftSum;
					leftBox.Grow(bin[i].bounds);
					leftArea[i] = leftBox.Area();

					rightSum += bin[BINS - 1 - i].primCount;
					rightCount[BINS - 2 - i] = rightSum;
					rightBox.Grow(bin[BINS - 1 - i].bounds);
					rightArea[BINS - 2 - i] = rightBox.Area();
				}
				
				// calculate SAH cost for the 7 planes
				scale = (boundsMax - boundsMin) / BINS;

				for (int i{}; i < BINS - 1; ++i)
				{
					const float planeCost = leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
					if (planeCost < bestCost)
					{
						axis = a;
						splitPos = boundsMin + scale * (i + 1);
						bestCost = planeCost;
					}
				}
			}

			return bestCost;
		}
		float CalculateNodeCost(BVHNode& node) const
		{
			const Vector3 e = node.aabbMax - node.aabbMin; // extent of the node
			const float surfaceArea = e.x * e.y + e.y * e.z + e.z * e.x;
			return node.primCount * surfaceArea;
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
		Ray(const Vector3& _origin, const Vector3& _direction)
			: origin{ _origin }
			, direction{ _direction }
			, reversedDirection{ Vector3{ 1.0f / _direction.x,  1.0f / _direction.y,  1.0f / _direction.z } } {}
		
		Ray(const Vector3& _origin, const Vector3& _direction, float _min, float _max)
			: origin{ _origin }
			, direction{ _direction }
			, reversedDirection{ Vector3{ 1.0f / _direction.x,  1.0f / _direction.y,  1.0f / _direction.z } }
			, min{ _min }
			, max{ _max } {}	
	

#ifdef USE_SIMD
		Ray() { O4 = D4 = rD4 = _mm_set1_ps(1); }
		union { struct { Vector3 origin; float dummy1; }; __m128 O4; };
		union { struct { Vector3 direction; float dummy2; }; __m128 D4; };
		union { struct { Vector3 rD; float dummy3; }; __m128 rD4; };
#else
		Vector3 origin{};
		Vector3 direction{};
		Vector3 reversedDirection{ 1 / direction.x, 1 / direction.y, 1 / direction.z };
#endif // USE_SIMD
		float min{ FLT_EPSILON };
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