#ifndef KDTREE_H_
#define KDTREE_H_

#include <pcl/common/geometry.h>

// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree() : root(NULL)
	{}

	void insert(PointT point, int id)
	{
		Node<PointT>** currentNode = &root;
		int depth = 0;
		float currentNodeComponent, targetComponent;

		while (*currentNode!=NULL)
		{
			switch(depth%3) {
			case 0:
				currentNodeComponent = (*currentNode)->point.x;
				targetComponent = point.x;
				break;
			case 1:
				currentNodeComponent = (*currentNode)->point.y;
				targetComponent = point.y;
				break;
			case 2:
				currentNodeComponent = (*currentNode)->point.z;
				targetComponent = point.z;
				break;
			}

			if (currentNodeComponent > targetComponent)
				currentNode = &(*currentNode)->right;
			else
				currentNode = &(*currentNode)->left;		

			depth++;	
		}
	
		*currentNode = new Node<PointT>(point, id);
	}

private:
	std::vector<int> searchHelper(Node<PointT>* currentNode, std::vector<int> &ids, PointT target, float distanceTol, uint depth=0)
	{
		float d = pcl::geometry::distance<PointT>(currentNode->point, target);
		float currentNodeComponent, targetComponent;

		switch(depth%3) {
			case 0:
				currentNodeComponent = currentNode->point.x;
				targetComponent = target.x;
				break;
			case 1:
				currentNodeComponent = currentNode->point.y;
				targetComponent = target.y;
				break;
			case 2:
				currentNodeComponent = currentNode->point.z;
				targetComponent = target.z;
				break;
		}


		if (currentNode!=NULL)
		{
			if (abs(currentNodeComponent - targetComponent) < distanceTol)
				if (d < distanceTol)
					ids.push_back(currentNode->id);

			depth++;
			if (currentNode->right!=NULL)
				ids = searchHelper(currentNode->right, ids, target, distanceTol, depth);
			if (currentNode->left!=NULL)
				ids = searchHelper(currentNode->left, ids, target, distanceTol, depth);
		}

		return ids;
	}

public:
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		ids = searchHelper(root, ids, target, distanceTol, 0);

		return ids;
	}
};

#endif




