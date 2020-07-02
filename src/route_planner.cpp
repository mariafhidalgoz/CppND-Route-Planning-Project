#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}


// CalculateHValue method.
// Node objects have a distance method to determine the distance to another node.
// Using the distance to the end_node for the h value.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return end_node->distance(*node);
}


// AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    // Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();

    for (auto neighbor : current_node->neighbors) {
        if (neighbor->visited != true) {
            // Set the parent, the h_value, the g_value for each node in current_node.neighbors.
            neighbor->parent = current_node;
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
            // Use the CalculateHValue below to implement the h-Value calculation.
            neighbor->h_value = CalculateHValue(neighbor);
            // Add the neighbor to open_list.
            open_list.push_back(neighbor);
            // Set the node's visited attribute to true.
            neighbor->visited = true;
        }
    }
}


// Sort the open_list according to the sum of the h value and g value.
bool Compare(RouteModel::Node *a, RouteModel::Node *b) {
    return a->g_value + a->h_value > b->g_value + b->h_value;
}

// NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);

    // Create a pointer to the node in the list with the lowest sum.
    RouteModel::Node *lowest_node = open_list.back();
    // Remove that node from the open_list.
    open_list.pop_back();
    // Return the pointer.
    return lowest_node;
}


// ConstructFinalPath method to return the final path found from your A* search.
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (1) {
        path_found.push_back(*current_node);
        if (current_node == start_node)
            break;

        auto parent_node = current_node->parent;
        // Add the distance from the node to its parent to the distance variable for each node in the chain.
        distance += current_node->distance(*parent_node);
        current_node = parent_node;
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    // Order vector. The start node should be the first element of the vector, the end node should be the last element.
    std::reverse(path_found.begin(),path_found.end());

    // The returned vector ordered
    return path_found;

}


// A* Search algorithm.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    open_list.push_back(start_node);
    current_node = start_node;
    current_node->visited = true;

    // Return the final path that was found, when the search has reached the end_node
    while (current_node != end_node){
        // Add all of the neighbors of the current node to the open_list.
        AddNeighbors(current_node);
        // Sort the open_list and return the next node.
        current_node = NextNode();
    }

    // Store the final path in the m_Model.path attribute. This path will then be displayed on the map tile.
    m_Model.path = ConstructFinalPath(end_node);

}