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
    bool useLess_ = true;
  	Node* lastNode; // used in search method

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
        root->split_dim = depth % 2;
        cnode = &root;
        return;
      }
	
      if  ((*cnode) == NULL)
      {
        *cnode = new Node(point, id); // assign a new node for the left or right node
        (*cnode)->split_dim = depth % 2;
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

   
  	// return target's box intersection results in the first element:
  	// 	1 - box locates in the left side
  	// 	2 - box lacates in the right side
  	// 	3 - box intersects both sides
  	int checkBoxIntersection(std::vector<float> target, Node* node, const float distanceTol, int split_dim)
    {
      int result = 0;

      float diff;
      diff = target[split_dim] - node->point[split_dim];
      
      // in the right side
      if (diff >=0) 
      {
        if (diff - distanceTol >= 0) result = 1; // right
        else result = 3; // in both sides
      }
      // in the left side
      else 
      {
        if (diff + distanceTol < 0) result = 2; // left
        else result = 3; // in both sides
      }
         
    return result;
    }
  
  	// return is the Node's point is in the box of target point
  bool isInBox(Node* node, std::vector<float> target, float distanceTol)
  {
    int dims = 2;
    int result = 1;
    
    //if (node == NULL) return false;
    // check for all dimensions
    
    for (int i = 0; i < dims; ++i)
    {
      if ((node->point[i] <= (target[i] + distanceTol)) && (node->point[i] >= (target[i] - distanceTol)))
      {
        	result *= 1;
      }
      else result *= 0;
    }

    return (bool)result;
  }
  
	// return a list of point ids in the tree that are within distance of target
  	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		Node* cnode = root;
      	std::vector<Node*> node_list = {cnode};
      	std::vector<int> ids;
      	//depth = 0;

      	while (node_list.size() > 0)
        {
          	//std::cout << "C" << endl;
          	if (*node_list.begin() == NULL) 
            {
              break;
            } else if (isInBox((*node_list.begin()), target, distanceTol)) 
            	{
              		ids.push_back((*node_list.begin())->id);
              		lastNode = (*node_list.begin()); // used when we need to know info about a particular point
            	}
          	//std::cout << "Split dim= " << (*node_list.begin())->split_dim << endl;
        	int check_res = checkBoxIntersection(target, *node_list.begin(), distanceTol, (*node_list.begin())->split_dim);
      		//std::cout << "A1" << endl;
            // check which node to open next
          	switch(check_res)
            {
                case 1: if ((*node_list.begin())->right != NULL) node_list.push_back((*node_list.begin())->right); break;
              	case 2: if ((*node_list.begin())->left != NULL) node_list.push_back((*node_list.begin())->left); break;  
               	case 3:
					//cout << node_list[0]->left << " " << node_list[0]->right << endl;
                	if ((*node_list.begin())->right != NULL && (*node_list.begin())->left != NULL) // left and right are not eq. to null
                    {
                      if (isInBox((*node_list.begin())->left, target, distanceTol))
                      { 
                        node_list.push_back((*node_list.begin())->right);
                        node_list.push_back((*node_list.begin())->left);
                      } else
                      {
                        node_list.push_back((*node_list.begin())->left);
                        node_list.push_back((*node_list.begin())->right);                      
                      }
                      break;
                    }
                	if ((*node_list.begin())->right != NULL) node_list.push_back((*node_list.begin())->right); break;
 					if ((*node_list.begin())->left != NULL) node_list.push_back((*node_list.begin())->left); break;  
                	break;
  
            }
			//std::cout << "B" << " " << node_list.size() << endl;
          node_list.erase(node_list.begin());
          //std::cout << "B1" << endl;

        }
    	//std::cout << "B2" << endl;
		return ids;
	}
};




