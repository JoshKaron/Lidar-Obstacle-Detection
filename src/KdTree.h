#ifndef KDTREE_H_
#define KDTREE_H_

#include <vector>
#include <cmath>
#include <pcl/common/common.h>

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	template<typename PointT>
	void setInputCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		for(int i = 0; i < cloud->size(); i++)
		{
			auto point = cloud->points[i];
			std::vector<float> vec = {point.x, point.y, point.z};
			insert(&root, 0, vec, i);
		}

	}

	void insert(Node** root, int depth, std::vector<float> point, int id)
	{
		if(*root == NULL)
		{
			*root = new Node(point, id);
		}
		else
		{
			int compIdx = depth % 3;

			if((*root)->point[compIdx] > point[compIdx])
			{
				insert(&((*root)->left), depth+1, point, id);
			}
			else
			{
				insert(&((*root)->right), depth+1, point, id);
			}
		}
		
	}

	void insert(std::vector<float> point, int id)
	{
		insert(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	template<typename PointT>
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<float> vec = {target.x, target.y, target.z};
		return search(vec, distanceTol);
	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchRec(root, 0, target, distanceTol, ids);
		return ids;
	}

	void searchRec(Node* node, int depth, std::vector<float> target, float disTol, std::vector<int>& ids)
	{
		if(node == NULL) { return; }

		int compIdx = depth % 3;

		bool xCheck = (node->point[0] >= target[0]-disTol) && (node->point[0] <= target[0]+disTol);
		bool yCheck = (node->point[1] >= target[1]-disTol) && (node->point[1] <= target[1]+disTol);
		bool zCheck = (node->point[2] >= target[2]-disTol) && (node->point[2] <= target[2]+disTol);
		
        if(xCheck && yCheck && zCheck)
		{
			float dis = sqrt(pow(node->point[0] - target[0],2) 
                           + pow(node->point[1] - target[1],2) 
                           + pow(node->point[2] - target[2],2));

			if(dis <= disTol)
			{
				ids.push_back(node->id);
			}
		}

		if(target[compIdx] - disTol < node->point[compIdx])
		{
			searchRec(node->left, depth+1, target, disTol, ids);
		}

	    if (target[compIdx] + disTol > node->point[compIdx])
		{
			searchRec(node->right, depth+1, target, disTol, ids);
		}
	}
	

};

#endif