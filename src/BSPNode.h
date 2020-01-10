// BSP node class for BSP trees
// Written by Dr. Sergey G. Kosov in 2019 for Jacobs University
#pragma once

#include "types.h"

class CBSPNode
{
public:
	/**
	 * @brief Leaf node constructor
	 * @param vpPrims The vector of pointers to the primitives included in the leaf node
	 */


	CBSPNode(const std::vector<std::shared_ptr<CPrim>>& vpPrims)
		: CBSPNode(vpPrims, 0, Vec3f::all(std::numeric_limits<float>::infinity()), Vec3f::all(-std::numeric_limits<float>::infinity()), nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr)
	{}
	/**
	 * @brief Branch node constructor
	 * @param splitVal The splitting value
	 * @param splitDim The splitting dimension
	 * @param left Pointer to the left sub-tree
	 * @param right Pointer to the right sub-tree
	 */

	CBSPNode(float size, Vec3f min, Vec3f max, std::shared_ptr<CBSPNode> tnr,std::shared_ptr<CBSPNode>tnl, std::shared_ptr<CBSPNode> tfl, 
	std::shared_ptr<CBSPNode>tfr, std::shared_ptr<CBSPNode> bnl, std::shared_ptr<CBSPNode>bfl, 
	std::shared_ptr<CBSPNode> bfr, std::shared_ptr<CBSPNode>bnr)
		: CBSPNode(std::nullopt, size, min, max, tnr, tnl, tfl, tfr, bnl, bfl, bfr, bnr)
	{}
	CBSPNode(const CBSPNode&) = delete;
	virtual ~CBSPNode(void) = default;
	const CBSPNode& operator=(const CBSPNode&) = delete;

	/**
	 * @brief Checks whether the node is either leaf or branch node
	 * @retval true if the node is the leaf-node
	 * @retval false if the node is a branch-node
	 */
	bool isLeaf(void) const {
		if (!m_tnr && ! m_tnl&& !m_bnr && ! m_tfl && !m_tfr && ! m_bnl && !m_bfr && ! m_bfl){
			return true;
		}
		return false;
	}
	
	/**
	 * @brief Traverses the ray \b ray and checks for intersection with a primitive
	 * @details If the intersection is found, \b ray.t is updated
	 * @param[in,out] ray The ray
	 * @param[in,out] t0 The distance from ray origin at which the ray enters the scene
	 * @param[in,out] t1 The distance from ray origin at which the ray leaves the scene
	 */

	bool traverse(Ray& ray, float t0, float t1){
		unsigned char a = 0;
		if (ray.dir[0] < 0){
			ray.org[0] = m_size - ray.org[0];
			ray.dir[0] = -ray.dir[0];
			a |= 4;
		}
		if (ray.dir[1] < 0){
			ray.org[1] = m_size - ray.org[1];
			ray.dir[1] = -ray.dir[1];
			a |= 2;
		}
		if (ray.dir[2] < 0){
			ray.org[2] = m_size - ray.org[2];
			ray.dir[2] = -ray.dir[2];
			a |= 4;
		}
		
		float tx0, tx1, ty0, ty1, tz0, tz1;

		tx0 = (m_min[0] - ray.org[0]) /ray.dir[0];
		tx1 = (m_max[0] - ray.org[0]) /ray.dir[0];
		ty0 = (m_min[1] - ray.org[1]) /ray.dir[1];
		ty1 = (m_max[1] - ray.org[1]) /ray.dir[1];
		tz0 = (m_min[2] - ray.org[2]) /ray.dir[2];
		tz1 = (m_max[2] - ray.org[2]) /ray.dir[2];

		if( fmax(fmax(tx0,ty0),tz0) < fmin(fmin(tx1,ty1),tz1) ){
    		return this->proc_subtree(a,tx0,ty0,tz0,tx1,ty1,tz1,ray,t0,t1);
		}
		return false;
	}

	int first_node(float tx0, float ty0, float tz0, float txm, float tym, float tzm){
		unsigned char answer = 0;   // initialize to 00000000
		// select the entry plane and set bits
		if(tx0 > ty0){
    		if(tx0 > tz0){ // PLANE YZ
        		if(tym < tx0) answer|=2;    // set bit at position 1
        		if(tzm < tx0) answer|=1;    // set bit at position 0
        		return (int) answer;
    		}
		}
		else {
    		if(ty0 > tz0){ // PLANE XZ
        		if(txm < ty0) answer|=4;    // set bit at position 2
        		if(tzm < ty0) answer|=1;    // set bit at position 0
        		return (int) answer;
   			}
		}
		// PLANE XY
		if(txm < tz0) answer|=4;    // set bit at position 2
		if(tym < tz0) answer|=2;    // set bit at position 1
		return (int) answer;
	}

	int new_node(float txm, int x, float tym, int y, float tzm, int z){
		if(txm < tym){
    		if(txm < tzm){return x;}  // YZ plane
		}
		else{
    		if(tym < tzm){return y;} // XZ plane
		}
		return z; // XY plane;
	}

	bool terminal(Ray& ray, float t0, float t1){
		for (auto pPrim : m_vpPrims)
				pPrim->Intersect(ray);
			return (ray.hit != NULL && ray.t < t1);
	}

	bool proc_subtree (unsigned int a, double tx0, double ty0, double tz0, double tx1, double ty1, double tz1, Ray& ray, float t0, float t1){
		float txm, tym, tzm;
		int currNode;
		std::shared_ptr<CBSPNode> children[8] = {m_bnl, m_bfl, m_tnl, m_tfl, m_bnr, m_bfr, m_tnr, m_tfr};

		if(tx1 < 0 || ty1 < 0 || tz1 < 0)
			return false;
		if (isLeaf())
			return terminal(ray, t0, t1);

		txm = 0.5*(tx0 + tx1);
		tym = 0.5*(ty0 + ty1);
		tzm = 0.5*(tz0 + tz1);

		currNode = first_node(tx0,ty0,tz0,txm,tym,tzm);

		do{
    		switch (currNode)
    		{
    			case 0:
        			children[a]->proc_subtree(a,tx0,ty0,tz0,txm,tym,tzm, ray, t0, t1);
        			currNode = new_node(txm,4,tym,2,tzm,1);
        			break;
    			case 1: 
        			children[1^a]->proc_subtree(a,tx0,ty0,tzm,txm,tym,tz1, ray, t0, t1);
        			currNode = new_node(txm,5,tym,3,tz1,8);
        			break;
    			case 2: 
        			children[2^a]->proc_subtree(a,tx0,tym,tz0,txm,ty1,tzm, ray, t0, t1);
        			currNode = new_node(txm,6,ty1,8,tzm,3);
        			break;
    			case 3: 
        			children[3^a]->proc_subtree(a,tx0,tym,tzm,txm,ty1,tz1, ray, t0, t1);
        			currNode = new_node(txm,7,ty1,8,tz1,8);
        			break;
    			case 4: 
        			children[4^a]->proc_subtree(a,txm,ty0,tz0,tx1,tym,tzm, ray, t0, t1);
        			currNode = new_node(tx1,8,tym,6,tzm,5);
        			break;
    			case 5: 
        			children[5^a]->proc_subtree(a,txm,ty0,tzm,tx1,tym,tz1, ray, t0, t1);
        			currNode = new_node(tx1,8,tym,7,tz1,8);
        			break;
    			case 6: 
        			children[6^a]->proc_subtree(a,txm,tym,tz0,tx1,ty1,tzm, ray, t0, t1);
        			currNode = new_node(tx1,8,ty1,8,tzm,7);
        			break;
    			case 7: 
    				children[7^a]->proc_subtree(a,txm,tym,tzm,tx1,ty1,tz1, ray, t0, t1);
        			currNode = 8;
        			break;
    		}
		} while (currNode<8);

		return false;
	}

	
private:

	CBSPNode(std::optional<std::vector<std::shared_ptr<CPrim>>> vpPrims, float size, Vec3f min, Vec3f max, std::shared_ptr<CBSPNode> tnr,
	 std::shared_ptr<CBSPNode>tnl, std::shared_ptr<CBSPNode> tfl, std::shared_ptr<CBSPNode>tfr, std::shared_ptr<CBSPNode> bnl, 
	 std::shared_ptr<CBSPNode>bfl, std::shared_ptr<CBSPNode> bfr, std::shared_ptr<CBSPNode>bnr)
		: m_size(size)
		, m_min(min)
		, m_max(max)
		, m_tnr(tnr)
		, m_tnl(tnl)
		, m_tfl(tfl)
		, m_tfr(tfr)
		, m_bnl(bnl)
		, m_bfl(bfl)
		, m_bfr(bfr)
		, m_bnr(bnr)
	{
		if (vpPrims) m_vpPrims = vpPrims.value();
	}
	
	
private:
	std::vector<std::shared_ptr<CPrim>> m_vpPrims;
	Vec3f m_min, m_max;
	float m_size;
	std::shared_ptr<CBSPNode> m_tnr;
	std::shared_ptr<CBSPNode> m_tnl;
	std::shared_ptr<CBSPNode> m_tfl;
	std::shared_ptr<CBSPNode> m_tfr;
	std::shared_ptr<CBSPNode> m_bnl;
	std::shared_ptr<CBSPNode> m_bfl;
	std::shared_ptr<CBSPNode> m_bfr;
	std::shared_ptr<CBSPNode> m_bnr;
};