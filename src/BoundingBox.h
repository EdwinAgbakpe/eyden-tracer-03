#pragma once

#include "types.h"
#include "ray.h"


namespace {
	inline Vec3f Min3f(const Vec3f a, const Vec3f b)
	{
		return Vec3f(MIN(a.val[0], b.val[0]), MIN(a.val[1], b.val[1]), MIN(a.val[2], b.val[2]));
	}

	inline Vec3f Max3f(const Vec3f a, const Vec3f b)
	{
		return Vec3f(MAX(a.val[0], b.val[0]), MAX(a.val[1], b.val[1]), MAX(a.val[2], b.val[2]));
	}
}

/**
 * @brief Bounding Box class
 */
class CBoundingBox
{
public:
	CBoundingBox(void) = default;
	~CBoundingBox(void)= default;
	
	/**
	 * @brief Resets the bounding box
	 * @details This function resets the member variables \b m_min and \b m_max
	 */
	void clear(void)
	{
		// m_min = Vec3f::all(std::numeric_limits<float>::infinity());
		// m_max = Vec3f::all(-std::numeric_limits<float>::infinity());
		org = Vec3f::all(std::numeric_limits<float>::infinity());
		m_side = 0;
	}
	
	/**
	 * @brief Extends the bounding box to contain point \b a
	 * @param a A point
	 */
	void extend(Vec3f a)
	{
		// m_min = Min3f(a, m_min);
		// m_max = Max3f(a, m_max);
		float temp;
		if (org == Vec3f::all(std::numeric_limits<float>::infinity()))
			org = a;
		temp = fmax(abs(a[0] - org[0]), (m_side/2));
		temp = fmax(abs(a[1] - org[1]), temp);
		temp = fmax(abs(a[2] - org[2]), temp);
		m_side = temp*2;
	}
	
	/**
	 * @brief Extends the bounding box to contain bounding box \b box
	 * @param box The second bounding box
	 */
	void extend(const CBoundingBox& box)
	{
		// extend(box.m_min);
		// extend(box.m_max);
		extend(box.org + Vec3f((box.m_side/2), (box.m_side/2), (box.m_side/2)));
		extend(box.org + Vec3f(-(box.m_side/2), -(box.m_side/2), -(box.m_side/2)));
	}
	
	/**
	 * @brief Checks if the current bounding box overlaps with the argument bounding box \b box
	 * @param box The secind bounding box to be checked with
	 */
	bool overlaps(const CBoundingBox& box)
	{
		Vec3f tr1, tr2, bl1, bl2;
		tr1 = org + Vec3f((m_side/2), (m_side/2), (m_side/2));
		bl1 = org + Vec3f(-(m_side/2), -(m_side/2), -(m_side/2));
		tr2 = box.org + Vec3f((box.m_side/2), (box.m_side/2), (box.m_side/2));
		bl2 = box.org + Vec3f(-(box.m_side/2), -(box.m_side/2), -(box.m_side/2));

		// std::cout<<tr2<<" "<<bl2<<std::endl;
		for (int i = 0; i < 3; i++) {
		if (!((bl1[i] < tr2[i]) && (tr1[i] > bl2[i]))){
				// std::cout<<"False ";
				return false;
			}
		}
		// std::cout<<"True ";
		return true;

		// for (int i = 0; i < 3; i++) {
		// 	if (!((bl1[i] < tr2[i]) && (tr1[i] > bl2[i]))){
		// 		std::cout<<"False ";
		// 		return false;
		// 	}
		// }
		// std::cout<<"True ";
		// return true;
		
		// return ((bl1[0] < tr2[0]) && (tr1[0] > bl2[0])) &&
		// 	   ((bl1[1] < tr2[1]) && (tr1[1] > bl2[1])) &&
		// 	   ((bl1[2] < tr2[2]) && (tr1[2] > bl2[2]));
	}
	
	/**
	 * @brief Clips the ray with the bounding box
	 * @param[in] ray The ray
	 * @param[in,out] t0 The distance from ray origin at which the ray enters the bounding box
	 * @param[in,out] t1 The distance from ray origin at which the ray leaves the bounding box
	 */
	void clip(const Ray& ray, float& t0, float& t1)
	{
		// std::cout<<"Clip ";
		float d, den;
		den = 1.0f / ray.dir.val[0];

		Vec3f tr, bl;
		tr = org + Vec3f((m_side/2), (m_side/2), (m_side/2));
		bl = org + Vec3f(-(m_side/2), -(m_side/2), -(m_side/2));

		if (ray.dir.val[0] > 0) {
			if ((d = (bl[0] - ray.org.val[0]) * den) > t0) t0 = d;
			if ((d = (tr[0] - ray.org.val[0]) * den) < t1) t1 = d;
		}
		else {
			if ((d = (tr[0] - ray.org.val[0]) * den) > t0) t0 = d;
			if ((d = (bl[0] - ray.org.val[0]) * den) < t1) t1 = d;
		}
		if (t0 > t1) return;

		den = 1.0f / ray.dir.val[1];
		if (ray.dir.val[1] > 0) {
			if ((d = (bl[1] - ray.org.val[1]) * den) > t0) t0 = d;
			if ((d = (tr[1] - ray.org.val[1]) * den) < t1) t1 = d;
		}
		else {
			if ((d = (tr[1] - ray.org.val[1]) * den) > t0) t0 = d;
			if ((d = (bl[1] - ray.org.val[1]) * den) < t1) t1 = d;
		}
		if (t0 > t1) return;

		den = 1.0f / ray.dir.val[2];
		if (ray.dir.val[2] > 0) {
			if ((d = (bl[2] - ray.org.val[2]) * den) > t0) t0 = d;
			if ((d = (tr[2] - ray.org.val[2]) * den) < t1) t1 = d;
		}
		else {
			if ((d = (tr[2] - ray.org.val[2]) * den) > t0) t0 = d;
			if ((d = (bl[2] - ray.org.val[2]) * den) < t1) t1 = d;
		}
		return;
	}
	
	
public:
	// Vec3f m_min = Vec3f::all(std::numeric_limits<float>::infinity());	///< The minimal point defying the size of the bounding box
	// Vec3f m_max = Vec3f::all(-std::numeric_limits<float>::infinity());	///< The maximal point defying the size of the bounding box
	Vec3f org = Vec3f::all(std::numeric_limits<float>::infinity());
	float m_side = 0;
};