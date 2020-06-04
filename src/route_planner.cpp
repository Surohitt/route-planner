#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model,
 float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Uses the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Stores the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
    
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node -> distance(*end_node); 
}


// The AddNeighbors method expands the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    current_node -> FindNeighbors();
    for (RouteModel::Node *Neighbor: current_node->neighbors){
        Neighbor->parent = current_node;
        Neighbor->h_value = CalculateHValue(Neighbor);
        Neighbor->g_value = current_node->g_value + current_node->distance(*Neighbor);

        open_list.push_back(Neighbor);
        Neighbor->visited = true;
    }

}


// The NextNode method sorts the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [](const auto & _1st, const auto & _2nd) {
        return _1st->h_value + _1st->g_value < _2nd->h_value + _2nd->g_value;
    });

    RouteModel::Node *lowestValNode = open_list.front();
    open_list.erase(open_list.begin());
    return lowestValNode;
}


// The ConstructFinalPath method returns the final path found from the A* search algorithm.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while(current_node->parent != nullptr){
        path_found.emplace_back(*current_node);
        distance += current_node->distance(*current_node -> parent);
        current_node = current_node->parent;
    }
    path_found.emplace_back(*current_node);

    reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm: 
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    start_node->visited = true;
    open_list.push_back(start_node);

    while (open_list.size()> 0){
        current_node = NextNode();

        if (current_node == end_node){
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }
        AddNeighbors(current_node);
    }
}