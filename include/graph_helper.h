#ifndef PYSKELETON2GRAPH_INCLUDE_GRAPH_HELPER_H_
#define PYSKELETON2GRAPH_INCLUDE_GRAPH_HELPER_H_

#include "typedef.h"
#include "graph.h"

template <typename ND, typename LD>
class GraphHelper
{
public:
    Graph<ND, LD> &graph;
    GraphHelper(){};
    GraphHelper(Graph<ND, LD> &graph) : graph(graph){};
    void addNodeWithHash(Hash hash, ND data)
    {
        Node<ND, LD> * node_ptr = graph.addNode(data);
        m_map_hash2node_ptr_.insert(std::pair<Hash, Node<ND, LD> *>(hash, node_ptr));
    }
    void addLinkWithHash(LD edge_attributes, Hash hash_source, Hash hash_destination)
    {
        graph.addLink(
            edge_attributes,
            m_map_hash2node_ptr_.at(hash_source),
            m_map_hash2node_ptr_.at(hash_destination));
    }

private:
    std::unordered_map<Hash, Node<ND, LD> *> m_map_hash2node_ptr_;
};

#endif //PYSKELETON2GRAPH_INCLUDE_GRAPH_HELPER_H_