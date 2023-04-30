#pragma once

#include <unordered_map>

#include "weighted-graph.hpp"
#include "graph-types.h"

template <typename T>
void computeIndegrees(const WeightedGraph<T> &graph, std::unordered_map<value_type<T>, int> &indegrees)
{
    // TODO store the indegree for each vertex in the graph in the indegrees map
    for (auto &vertex : graph)
        indegrees[vertex.first] = 0;

    for (auto &adj_list : graph)
    {
        for (auto &vertex : adj_list.second)
            indegrees[vertex.first] += 1;
    }
}
