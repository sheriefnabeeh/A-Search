#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    RouteModel::Node node;
    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    RoutePlanner::start_node = &m_Model.FindClosestNode(start_x,start_y);
    RoutePlanner::end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    
    double distance_to_end = RoutePlanner::end_node->distance(*node);
    //for tracking :
    //std::cout << "distace to end : " << distance_to_end << "\n";
    return distance_to_end;
}
bool CompareNodes(const RouteModel::Node* fnode, const RouteModel::Node* snode) {
    return (fnode->g_value + fnode->h_value) > (snode->g_value + snode->h_value);
    //i should check if h = 0;
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    current_node->FindNeighbors();

    for (RouteModel::Node* neighbor : current_node->neighbors) {
       
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
            neighbor->parent = current_node;
            neighbor->visited = true;
            open_list.push_back(neighbor);
            

    }

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.


RouteModel::Node *RoutePlanner::NextNode() {
   
    //initialize to a random val
    //float lowest_sum = 0.0f;
    std::sort(open_list.begin(), open_list.end(), CompareNodes);
    RouteModel::Node *nearest_node = open_list.back();
    open_list.pop_back();
    return nearest_node ;
}



std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node *found_node = current_node;
  // std::cout << "start node x : " << RoutePlanner::start_node->x << " and y : " << RoutePlanner::start_node->y << "\n";
  //  std::cout << "end node x : " << RoutePlanner::end_node->x << " and y : " << RoutePlanner::end_node->y << "\n" << "\n";
   RouteModel::Node* parent = current_node;
    path_found.push_back(*current_node);
    while ((*parent).x !=RoutePlanner::start_node->x){

         parent = found_node->parent;
        distance += (*found_node).distance(*parent);
        path_found.push_back(*parent);
        found_node = parent; 
        

    }
    std::reverse(path_found.begin(), path_found.end());
    // for (RouteModel::Node node : path_found) {
    //     std::cout << "path node x : " << node.x<< " and y : " << node.y << "\n";
    // }
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node* current_node =  RoutePlanner::start_node;
    open_list.push_back(current_node);
    current_node->visited = true;
    while (current_node != RoutePlanner::end_node) {
        AddNeighbors(current_node);
        current_node = NextNode();
    }
        //  std::cout << "distance between start and end :" << RoutePlanner::start_node->distance(*RoutePlanner::end_node);
        m_Model.path = ConstructFinalPath(current_node);
 }