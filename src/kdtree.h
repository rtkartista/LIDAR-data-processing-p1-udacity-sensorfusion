#ifndef KDTREE_H
#define KDTREE_H

#include "render/render.h"
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

	void inserthelper(Node** node, int depth, std::vector<float> point, int id)
	{
		// if node is empty
		if (*node == NULL) 
		{
			*node = new Node(point, id);
			return;
    	}
		else
		{
			uint cd = depth % point.size();
			if(point[cd]<((*node)->point[cd]))
				inserthelper(&((*node)->left),depth+1,point,id);
			else
				inserthelper(&((*node)->right),depth+1,point,id);
				
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		// recording depth as we transverse the tree
		inserthelper(&root, 0, point, id);
	}

	// return a list of point ids in the tree that are within distance of target
	void searchhelpers(std::vector<int> &ids, std::vector<float> target, float distanceTol, Node** node, int depth)
	{
		if(*node != NULL)
		{
			// if the node point inside the bounding box around target
			int check =1;
			for (int i = 0; i < target.size(); i++)
			{
				check &= std::abs((*node)->point[i]-target[i])<=distanceTol;
				//std::cout<<check<<std::endl;
			}
			//std::cout<<target.size()<<std::endl;
			if (check == true)
			{
				float k=0;
				for (int i = 0; i < target.size(); i++)
				{
					k = k + ((*node)->point[i] - target[i])*((*node)->point[i] - target[i]);
				}
				
				float dist = sqrt(k);
				if (dist<=distanceTol)
				{
					//std::cout<<(*node)->id<<std::endl;
					ids.push_back((*node)->id);
				}
			}

			// if the node is outside the bounding box - begin recursion
			// the target point box might lie in both the division 
			uint cd = depth%target.size();

			if((target[cd]-distanceTol)<((*node)->point[cd]))
				searchhelpers(ids, target, distanceTol, &((*node)->left), depth+1);
			if((target[cd]+distanceTol)>((*node)->point[cd]))
				searchhelpers(ids, target, distanceTol, &((*node)->right), depth+1);	
		}
	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids_int;
		searchhelpers(ids_int, target, distanceTol, &root, 0);

		return ids_int;
	}
	

};
#endif
