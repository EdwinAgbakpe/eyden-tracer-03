#pragma once

#include "BSPNode.h"
#include "BoundingBox.h"

namespace {
	inline int MaxDim(Vec3f v)
	{
		return (v.val[0] > v.val[1]) ? ((v.val[0] > v.val[2]) ? 0 : 2) : ((v.val[1] > v.val[2]) ? 1 : 2);
	}
}

class BSPTree
{
public:
	/**
	 * @brief Constructor
	 * @param bounds The scene bounding box
	 * @param vpPrims The vector of pointers to the primitives in the scene
	 */
	BSPTree(CBoundingBox& bounds, const std::vector<std::shared_ptr<CPrim>>& vpPrims)
		: m_bounds(bounds)
		, m_maxDepth(0)
		, m_minTri(3)
		, m_root(nullptr)
	{
		m_root = BuildTree(bounds, vpPrims, 0);
	}
	
	/**
	 * @brief Builds the BSP tree
	 * @details This function builds the BSP tree recursively
	 * @param box The bounding box containing all the scene primitives
	 * @param vpPrims The vector of pointers to the primitives included in the bounding box \b box
	 * @param depth The distance from the root node of the tree
	 */
	std::shared_ptr<CBSPNode> BuildTree(const CBoundingBox& box, const std::vector<std::shared_ptr<CPrim>>& vpPrims, int depth)
	{
		if (depth > m_maxDepth || vpPrims.size() <= m_minTri) {
			// could do some optimizations here..
			return std::make_shared<CBSPNode>(vpPrims);
		}
		if (vpPrims.size() == 0){
			std::cout<<"We shouldn't even get here"<<std::endl;
			return std::make_shared<CBSPNode>(vpPrims);
		}

		Vec3f tr, bl;
		tr = box.org + Vec3f((box.m_side/2), (box.m_side/2), (box.m_side/2));
		bl = box.org + Vec3f(-(box.m_side/2), -(box.m_side/2), -(box.m_side/2));
		
		Vec3f diam = tr-bl;

		int splitDim = MaxDim(diam);

		// CBoundingBox lBounds = box;
		// CBoundingBox rBounds = box;

		float splitVal = /*lBounds.m_max[splitDim] = rBounds.m_min[splitDim] =*/ (bl[splitDim] + tr[splitDim]) * 0.5f;
		// std::vector<std::shared_ptr<CPrim>> lPrim;
		// std::vector<std::shared_ptr<CPrim>> rPrim;


		// for (auto pPrim : vpPrims) {
		// 	if (pPrim->inVoxel(lBounds))
		// 		lPrim.push_back(pPrim);
		// 	if (pPrim->inVoxel(rBounds))
		// 		rPrim.push_back(pPrim);
		// }

		// auto pLeft = BuildTree(lBounds, lPrim, depth + 1);
		// auto pRight = BuildTree(rBounds, rPrim, depth + 1);

		CBoundingBox tnr, tnl, tfl, tfr, bnl, bfl, bfr, bnr;
		tnr.org = box.org + Vec3f((box.m_side/4), (box.m_side/4), (box.m_side/4));
		tnl.org = box.org + Vec3f(-(box.m_side/4), (box.m_side/4), (box.m_side/4));
		tfl.org = box.org + Vec3f(-(box.m_side/4), -(box.m_side/4), (box.m_side/4));
		tfr.org = box.org + Vec3f((box.m_side/4), -(box.m_side/4), (box.m_side/4));
		bnl.org = box.org + Vec3f(-(box.m_side/4), (box.m_side/4), -(box.m_side/4));
		bfl.org = box.org + Vec3f(-(box.m_side/4), -(box.m_side/4), -(box.m_side/4));
		bfr.org = box.org + Vec3f((box.m_side/4), -(box.m_side/4), -(box.m_side/4));
		bnr.org = box.org + Vec3f((box.m_side/4), (box.m_side/4), -(box.m_side/4));

		tnr.m_side = tnl.m_side = tfl.m_side = tfr.m_side = bnl.m_side = bfl.m_side = bfr.m_side = bnr.m_side = box.m_side * 0.5f;
		std::vector<std::shared_ptr<CPrim>> tnrPrim;
		std::vector<std::shared_ptr<CPrim>> tnlPrim;
		std::vector<std::shared_ptr<CPrim>> tflPrim;
		std::vector<std::shared_ptr<CPrim>> tfrPrim;
		std::vector<std::shared_ptr<CPrim>> bnlPrim;
		std::vector<std::shared_ptr<CPrim>> bflPrim;
		std::vector<std::shared_ptr<CPrim>> bfrPrim;
		std::vector<std::shared_ptr<CPrim>> bnrPrim;

		for (auto pPrim : vpPrims) {
			if (pPrim->inVoxel(tnr))
				tnrPrim.push_back(pPrim);
			if (pPrim->inVoxel(tnl))
				tnlPrim.push_back(pPrim);
			if (pPrim->inVoxel(tfl))
				tflPrim.push_back(pPrim);
			if (pPrim->inVoxel(tfr))
				tfrPrim.push_back(pPrim);
			if (pPrim->inVoxel(bnl))
				bnlPrim.push_back(pPrim);
			if (pPrim->inVoxel(bfl))
				bflPrim.push_back(pPrim);
			if (pPrim->inVoxel(bfr))
				bfrPrim.push_back(pPrim);
			if (pPrim->inVoxel(bnr))
				bnrPrim.push_back(pPrim);
		}


		auto ptnr = BuildTree(tnr, tnrPrim, depth + 1);
		auto ptnl = BuildTree(tnl, tnlPrim, depth + 1);
		auto ptfl = BuildTree(tfl, tflPrim, depth + 1);
		auto ptfr = BuildTree(tfr, tfrPrim, depth + 1);
		auto pbnl = BuildTree(bnl, bnlPrim, depth + 1);
		auto pbfl = BuildTree(bfl, bflPrim, depth + 1);
		auto pbfr = BuildTree(bfr, bfrPrim, depth + 1);
		auto pbnr = BuildTree(bnr, bnrPrim, depth + 1);
		std::cout<<vpPrims.size()<<" : ";
		std::cout<<tnrPrim.size()+tnlPrim.size()+tflPrim.size()+tfrPrim.size()+bnlPrim.size()+bflPrim.size()+bfrPrim.size()+bnrPrim.size()<<std::endl;

		return std::make_shared<CBSPNode>(box.m_side, bl, tr, ptnr, ptnl, ptfl, ptfr, pbnl, pbfl, pbfr, pbnr);
	}

	/**
	 * @brief Checks whether the ray \b ray intersects a primitive.
	 * @details If ray \b ray intersects a primitive, the \b ray.t value will be updated
	 * @param[in,out] ray The ray
	 */
	bool Intersect(Ray& ray)
	{
		// std::cout<<"Intersect ";
		
		float t0 = 0;
		float t1 = ray.t;


		m_bounds.clip(ray, t0, t1);
		
		if (t1 - t0 < Epsilon)
			return false;

		m_root->traverse(ray, t0, t1);
		if (ray.hit)
			return true;

		return false;
	}

	
private:
	CBoundingBox 				m_bounds;
	const int 	 				m_maxDepth;
	const size_t	 			m_minTri;
	std::shared_ptr<CBSPNode> 	m_root;
};

