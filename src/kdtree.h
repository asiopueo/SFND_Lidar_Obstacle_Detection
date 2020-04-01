#include "../../render/render.h"


template<typename PointT>
inline float euclDist(PointT X, PointT Y) {
	return sqrt(pow((X[0]-Y[0]),2) + pow((X[1]-Y[1]),2));
}


// Structure to represent node of kd tree
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree
{
	Node* root;

	KdTree() : root(NULL)
	{}

	void insert(PointT point, int id)
	{
		Node** currentNode = &root;
		int depth = 0;
		
		while (*currentNode!=NULL)
		{
			if ((*currentNode)->point[depth%3] > point[depth%3])
				currentNode = &(*currentNode)->right;
			else
				currentNode = &(*currentNode)->left;		

			depth++;	
		}
	
		*currentNode = new Node(point, id);
	}

private:
    template<typename PointT>
	std::vector<int> searchHelper(Node* currentNode, std::vector<int> &ids, PointT target, float distanceTol, uint depth=0)
	{
		float d = euclDist(currentNode->point, target);

		if (currentNode!=NULL)
		{
			if (abs(currentNode->point[depth3] - target[depth%3]) < distanceTol)
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
    template<typename PointT>
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		ids = searchHelper(root, ids, target, distanceTol, 0);

		return ids;
	}
};




