/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;
  	int split_dim;
  	bool processed = false; // used in clustering algorithm only

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;
  	Node** cnode; // the current node
  	int depth = 0;
  	Node* lastNode; // used in search method

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
	  int max_dims = point.size();      

	  if (root == NULL) 
      {
        root = new Node(point, id);
        root->split_dim = 0;
        cnode = &root;
        return;
      }
		

	  // the expression for returning from recursion 
      if  ((*cnode) == NULL)
      {
        *cnode = new Node(point, id); // assign a new node for the left or right node
        (*cnode)->split_dim = depth % max_dims;
        cnode = &root; // set to root node 
        depth = 0; // reset the depth counter
        return;
      }
      
      
      // compare value and choose the dimension to split
	  int dim = depth % max_dims;
	  if (point[dim] < (*cnode)->point[dim])
        cnode = &(*cnode)->left;
      else
        cnode = &(*cnode)->right;
      
      // go recursively in the insert function.
	  depth++;
      insert(point, id);
      return;
      
	}

	void checkIntersection(Node* node, std::vector<float> target, float distanceTol, std::vector<int>* ids)
	{

		if (node == NULL) // return from the recursion
		{
			return;
		}

		// check is the node's point within the box (near) of the target
		bool push = true;
		for (int i = 0; i < target.size(); ++i) // check for all dimensions 
		{
			if (node->point[i] >= (target[i] - distanceTol) && node->point[i] <= (target[i] + distanceTol))
			{
				push *= true;
			} else push *= false;
		}
		if (push) 
		{
			ids->push_back(node->id);
			cnode = &node;
		}

		// h node to open as the next
		int split_dim = node->split_dim;
		std::vector<int> new_ids;
		// left:
		if ((target[split_dim] - distanceTol) < node->point[split_dim] && (target[split_dim] + distanceTol) < node->point[split_dim])
		{
			checkIntersection(node->left, target, distanceTol, ids);
		}
		// right:
		if ((target[split_dim] - distanceTol) >= node->point[split_dim] && (target[split_dim] + distanceTol) > node->point[split_dim])
		{
			checkIntersection(node->right, target, distanceTol, ids);
		}
		// both:

		if ((target[split_dim] - distanceTol) < node->point[split_dim] && (target[split_dim] + distanceTol) > node->point[split_dim])
		{
			checkIntersection(node->left, target, distanceTol, ids);	
			checkIntersection(node->right, target, distanceTol, ids);
		}

		return;
	}

	// return a list of point ids in the tree that are within distance of target
  	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
      	std::vector<int>* ids (new std::vector<int>);
		Node* currNode = root;
		
		checkIntersection(currNode, target, distanceTol, ids);
		return *ids;	
	}

};




