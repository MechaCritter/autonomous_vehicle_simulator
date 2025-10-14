//
// Created by vunha on 4/20/2025.
//

#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <iostream>
#include <filesystem>
#include <array>
#include <include/CXXGraph/CXXGraph.hpp>


using namespace CXXGraph;


/*
 *  @brief a small wrapper that helps organize graph data better.
 *
 *  @note: since CXXGraph's node finder using an ID iterates over the
 *  whole node set everytime, it's better to just save a copy of
 *  these nodes in the map object to avoid O(n) searches everytime.
 */
struct GraphData {
    Graph<int> graph;
    std::unordered_map<std::string, std::shared_ptr<Node<int>>> graph_nodes{}; // maps node labels to node pointers
    std::unordered_map<int, std::shared_ptr<Edge<int>>> graph_edges{}; // maps edge ids to edge pointers
    std::unordered_map<std::string, b2Vec2> node_coords{}; // maps node labels to coordinates
};


namespace utils {
    /**
    *  @brief Finds the scalar interval [mn, mx] of a convex polygon projected onto
    *  the given axis.
    * @detail for each point p in pts, compute the dot product p·axis and track the min and max values.
    *
    * @note the axis should be normalized (unit length) for correct results.
    *
    * @returns A pair {mn, mx} representing the projection interval.
    */
    inline std::pair<float, float> projectMinMaxPolygon(const std::array<b2Vec2,4>& pts, const b2Vec2& axis){
        float mn = std::numeric_limits<float>::infinity();
        float mx = -mn;
        for (const auto&[x, y]: pts) {
            const float v = x*axis.x + y*axis.y;
            mn = std::min(mn, v); mx = std::max(mx, v);
        }
        return {mn, mx};
    }

    /**
     * @brief converts node id to label string
     */
    inline std::string nodeIdToLabel(int new_id) {
        return std::to_string(new_id);
    }

    /**
     * @brief inserts a new node into the graph. Then, add the node to the `graph_nodes` and
     * `node_coords` hashmaps.
     *
     * @note the label is simply string(node_id)
     *
     *  @param graph_data the graph data structure containing the graph and its nodes/edges
     *  @param coord the 2D coordinate of the new node
     *  @return The pointer to the newly inserted node
     */
    inline std::shared_ptr<Node<int>> insertNewNodeReturnPtr(GraphData* graph_data, const b2Vec2& coord) {
        size_t new_id = graph_data->graph.getNodeSet().size();
        std::string label = nodeIdToLabel(new_id);
        auto new_node = std::make_shared<Node<int>>(label, new_id);
        graph_data->graph.addNode(new_node);
        graph_data->graph_nodes[label] = new_node;
        graph_data->node_coords[label] = coord;
        return new_node;
    }

    /**
     * @brief inserts a new node into the graph. Then, add the node to the `graph_nodes` and
     * `node_coords` hashmaps.
     *
     *  @param graph_data the graph data structure containing the graph and its nodes/edges
     *  @param coord the 2D coordinate of the new node
     *  @return The id of the inserted node
     */
    inline int insertNewNodeReturnID(GraphData* graph_data, const b2Vec2& coord) {
        size_t new_id = graph_data->graph.getNodeSet().size();
        std::string label = nodeIdToLabel(new_id);
        auto new_node = std::make_shared<Node<int>>(label, new_id);
        graph_data->graph.addNode(new_node);
        graph_data->graph_nodes[label] = new_node;
        graph_data->node_coords[label] = coord;
        return new_id;
    }

    /**
     * @brief inserts an undirected edge (bidirectional) between two nodes into the graph. Then,
     * add the edge to the `graph_edges` hashmap.
     *
     *  @param graph_data: the graph data structure containing the graph and its nodes/edges
     *  @param first_node: pointer to first node
     *  @param second_node: pointer to second node
     *  @param weight: the weight of the edge (default is 1.0)
     *
     *  @return the pointer of the newly inserted edge
    */
    inline std::shared_ptr<Edge<int>> insertUndirectedEdgeReturnPtr(GraphData* graph_data,
                                                const std::shared_ptr<Node<int>> &first_node,
                                                const std::shared_ptr<Node<int>> &second_node,
                                                float weight = 1.0f) {
        // TODO: check first if the nodes are in the graph
        std::pair node_pair = {first_node, second_node};
        int edge_id = graph_data->graph.getEdgeSet().size();
        auto edge = std::make_shared<UndirectedWeightedEdge<int>>(edge_id, node_pair, weight);
        graph_data->graph.addEdge(edge);
        graph_data->graph_edges[edge_id] = edge;
        return edge;
    }

    /**
     * @brief inserts a directed edge (unidirectional) between two nodes into the graph. Then,
     * add the edge to the `graph_edges` hashmap.
     *
     *  @param graph_data: the graph data structure containing the graph and its nodes/edges
     *  @param from_node: pointer to the starting node
     *  @param to_node: pointer to the ending node
     *  @param weight: the weight of the edge (default is 1.0)
     *
     *  @return the pointer of the newly inserted edge
    */
    inline std::shared_ptr<Edge<int>> insertDirectedEdgeReturnPtr(GraphData* graph_data,
                                                const std::shared_ptr<Node<int>> &from_node,
                                                const std::shared_ptr<Node<int>> &to_node,
                                                float weight = 1.0f) {
        std::pair node_pair = {from_node, to_node};
        int edge_id = graph_data->graph.getEdgeSet().size();
        auto edge = std::make_shared<DirectedWeightedEdge<int>>(edge_id, node_pair, weight);
        graph_data->graph.addEdge(edge);
        graph_data->graph_edges[edge_id] = edge;
        return edge;
    }
} // namespace utils

#endif //UTILS_H
