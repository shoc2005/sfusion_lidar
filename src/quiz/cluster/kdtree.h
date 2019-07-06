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

  // define own comparator 
  bool mycmp(float lhs, float rhs)
   {
      if ( useLess_ )
      {
         return (lhs <= rhs);
      }
      else
      {
         return (lhs >= rhs);
      }
   }
  
  	// return target's box intersection results in the first element:
  	// 	0 - the node have both sides equal to NULL
  	// 	1 - box locates in the left side
  	// 	2 - box lacates in the right side
  	// 	3 - box intersects both sides
  	int checkBoxIntersection(std::vector<float> target, Node* node, float distanceTol, int split_dim)
    {
      int result = 0;
      if (node->left == NULL && node->right == NULL) return result;
      if (node->left == NULL) return 2;
      if (node->right == NULL) return 1;
      
      // chose a side
      if (target[split_dim] >= node->point[split_dim])
      {
        	distanceTol *= -1;
        	useLess_ = false;
      } else useLess_ = true;
      
      if (mycmp(target[split_dim] + distanceTol, node->point[split_dim])) result = 1; // the box is located in the left
      	else
      	{
        	if (mycmp(target[split_dim] - distanceTol, node->point[split_dim])) result = 3;  // the box also is located in both sides
          		else
                {
                  result = 2; // the box is only located in the right
                }
      	}
    
   	// check the node's point is in the box 
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

      	while (node_list.size() > 0)
        {
          	if (*node_list.begin() == NULL) 
            {
              node_list.erase(node_list.begin());
              break;
            } else if (isInBox(node_list[0], target, distanceTol)) ids.push_back(node_list[0]->id);
          	
        	int check_res = checkBoxIntersection(target, *node_list.begin(), distanceTol, (*node_list.begin())->split_dim);
      
            // check which node to open next
          	switch(check_res)
            {
                case 1: node_list.push_back(node_list[0]->left); break;
              	case 2: node_list.push_back(node_list[0]->right); break;  
               	case 3:
					//cout << node_list[0]->left << " " << node_list[0]->right << endl;
              		if (isInBox(node_list[0]->left, target, distanceTol))
                    { 
                      node_list.push_back(node_list[0]->left);
                      node_list.push_back(node_list[0]->right);
                    } else
                    {
                      node_list.push_back(node_list[0]->right);
                      node_list.push_back(node_list[0]->left);                      
                    }

                	break;
  
            }

          node_list.erase(node_list.begin());

        }
    
		return ids;
	}
};




