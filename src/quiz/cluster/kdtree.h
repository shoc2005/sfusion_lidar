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

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;
  	Node** cnode; // the current node
  	int depth = 0;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
      if (root == NULL) 
      {
        root = new Node(point, id);
        cnode = &root;
        return;
      }
	
      if  ((*cnode) == NULL)
      {
        *cnode = new Node(point, id); // assign a new node for the left or right node
        cnode = &root;
        depth = 0; // reset the depth counter
        return;
      }
      
      // check which dimension neet to be compared
      int dim = 0;
      if (depth % 2 != 0) dim = 1;
      
      ++depth;
      
      // compare value
	  if (point[dim] < (*cnode)->point[dim])
        cnode = &(*cnode)->left;
      else
        cnode = &(*cnode)->right;
      
      // go recursively in the insert function.
      insert(point, id);
      return;
      
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
	

};




