#pragma once


#include "RTTriangle.hpp"
#include "RaycastResult.hpp"
#include "rtlib.hpp"
#include "Bvh.hpp"

#include "base/String.hpp"

#include <vector>
#include <atomic>

namespace FW
{


// Given a vector n, forms an orthogonal matrix with n as the last column, i.e.,
// a coordinate system aligned such that n is its local z axis.
// You'll have to fill in the implementation for this.
Mat3f formBasis(const Vec3f& n);

Vec2f getTexelCoords(Vec2f uv, const Vec2i size);


// Main class for tracing rays using BVHs.
class RayTracer {
public:
                        RayTracer				(void);
                        ~RayTracer				(void);

						void					constructHierarchy(std::vector<RTTriangle>& triangles, SplitMode splitMode);

    void				saveHierarchy			(const char* filename, const std::vector<RTTriangle>& triangles);
    void				loadHierarchy			(const char* filename, std::vector<RTTriangle>& triangles);

    RaycastResult		raycast					(const Vec3f& orig, const Vec3f& dir) const;

    // This function computes an MD5 checksum of the input scene data,
    // WITH the assumption that all vertices are allocated in one big chunk.
    static FW::String	computeMD5				(const std::vector<Vec3f>& vertices);

    std::vector<RTTriangle>* m_triangles;

	void resetRayCounter() { m_rayCount = 0; }
	int getRayCount() { return m_rayCount; }

private:
	mutable std::atomic<int> m_rayCount;
    // YOUR CODE HERE (R1):
    // This is the library implementation of the ray tracer.
    // Remove this once you have integrated your own ray tracer.
    std::unique_ptr<rtlib::RayTracer> m_rt; 
};


} // namespace FW