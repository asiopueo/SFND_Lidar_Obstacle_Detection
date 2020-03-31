/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


inline float euclDist(std::vector<float> X, std::vector<float> Y) {
	return sqrt(pow((X[0]-Y[0]),2) + pow((X[1]-Y[1]),2));
}


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

	KdTree() : root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly within the root 
		// Watch out: a two-dimensional vector is a vector of size 2!
		
		Node** currentNode = &root;
		int depth = 0;
		
		while (*currentNode!=NULL)
		{
			if ((*currentNode)->point[depth%2] > point[depth%2])
				currentNode = &(*currentNode)->right;
			else
				currentNode = &(*currentNode)->left;		

			depth++;	
		}
	
		*currentNode = new Node(point, id);
		std::cout << "Inserted point no. " << id << ": (" << point[0] << "," << point[1] << ") into kd-tree at " << currentNode << std::endl;
	}

private:
	std::vector<int> searchHelper(Node* currentNode, std::vector<int> &ids, std::vector<float> target, float distanceTol, uint depth=0)
	{
		float d = euclDist(currentNode->point, target);

		if (currentNode!=NULL)
		{
			if (abs(currentNode->point[depth%2] - target[depth%2]) < distanceTol)
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
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		ids = searchHelper(root, ids, target, distanceTol, 0);

		return ids;
	}
};




