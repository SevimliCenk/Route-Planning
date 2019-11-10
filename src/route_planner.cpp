#include "route_planner.h"
#include <algorithm>


//// Question: why model object is created with reference ?
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.

    this->start_node = &m_Model.FindClosestNode(start_x, start_y); //// error: red under the m_model
    this->end_node = &m_Model.FindClosestNode(end_x, end_y); //// error: red under the m_model

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

//// Question: why Node is const and pointer?
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float h_value = node->distance(*end_node); //// error: red under the  this pointer (end:node is RoutePlanner object, so sent it with this pointer)
    return h_value;

}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

     // Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
     current_node->FindNeighbors(); 

      // For each node in current_node.neighbors, set the parent, the h_value, the g_value.  
      for (int i=0; i< current_node->neighbors.size(); i++) {

          // created neighbor_Node Object
          RouteModel::Node *neighbor_node = current_node->neighbors[i] ; //// error: red under the current_node

          //For each neighbor of the current_node, parent of that neighbor is set to the current_node
          neighbor_node->parent = current_node;

          // calculate neighbor_node s H_value  
          neighbor_node->h_value = RoutePlanner::CalculateHValue(current_node); 

          // g value is the distance between current node and neighbor node
          neighbor_node->g_value = current_node->distance(*neighbor_node); 

         // neighbor_node added into the open_list. 
         open_list.push_back(neighbor_node); //// error: red under the open_list 

         // set the node's visited attribute to true.
         neighbor_node->visited = true; 
          
      }

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

//// Question: why RoutePlanner Function is returned pointer?
RouteModel::Node *RoutePlanner::NextNode() {

    //Sort the open_list according to the sum of the h value and g value. For that 2 helper functions are defined: Compare and NodeSort
    NodeSort(&open_list);

    //Create a pointer to the node in the list with the lowest sum
    RouteModel::Node *next_node = open_list.back(); 

    // Remove that node from the open_list.
    open_list.pop_back();

    return next_node; //// Question: why not *next_node?
}


//created 2 helper functions 
void RoutePlanner::NodeSort(std::vector<RouteModel::Node*> *node) { 
    std::sort(node->begin(), node->end(), Compare);
}

bool Compare(RouteModel::Node* n1, RouteModel::Node* n2) { 
      float gh1 = n1->g_value + n1->h_value; 
      float gh2 = n2->g_value + n2->h_value; 
    return (gh1 > gh2);
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.


//// Question: Why it is taking the current_node as pointer?
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    
    // Final Node sent into the Node_Vector
    path_found.push_back(*current_node); //// error: red under the path_found

    // while nect parent is not the start point
    while(current_node != start_node) {

        // the distance from the node to its parent
        distance += current_node->distance(*current_node->parent);  //// error: red under the current_node

        // parent of the current_node is assigned as current node.
        current_node = current_node->parent;

        path_found.push_back(*current_node); //// error: red under the path_found
    }
    
    // vector oder is reversed and corrected
    std::reverse(std::begin(path_found), std::end(path_found));

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here. 
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    current_node->visited = true

    // TODO: Implement your solution here.

    //// Question: nullptr is sent into open_list, ist that right?
    open_list.push_back(current_node);

    while(open_list.size()>0) {

        // add all of the neighbors of the current node to the open_list.
        AddNeighbors(current_node); //// Question: Should I sent it with reference? (&current_node), because ConstructFinalPath(RouteModel::Node *current_node)

        // sort the open_list and return the next node, which is new current_node
        current_node = NextNode();
    }

    // return the final path that was found.
    std::vector<RouteModel::Node> final_path = ConstructFinalPath(current_node);
     
    //// Question:: is that the right method to Store the final path in the m_Model.path attribute or there is a better solution?
    m_Model.path = final_path;

}