#ifndef PYSKELETON2GRAPH_INCLUDE_GRAPH_HELPER_H_
#define PYSKELETON2GRAPH_INCLUDE_GRAPH_HELPER_H_

#include <map>
#include "typedef.h"
#include "graph.h"

template <typename ND, typename LD>
class GraphHelper
{
public:
    Graph<ND, LD> &graph;

    GraphHelper(){};
    GraphHelper(Graph<ND, LD> &graph) : graph(graph){};
    void addNode(Hash hash, ND data)
    {
        Node<ND, LD> *node_ptr = graph.addNode(data);
        m_map_hash2node_ptr_.insert(std::pair<Hash, Node<ND, LD> *>(hash, node_ptr));
        m_map_node_ptr2hash_.insert(std::pair<Node<ND, LD> *, Hash>(node_ptr, hash));
        //std::cout << data.getHash() << " " << m_map_hash2node_ptr_.at(hash)->data.getHash() << std::endl;
    }

    void removeNode(const Hash &hash)
    {
        Node<ND, LD> *node_ptr = m_map_hash2node_ptr_.at(hash);
        m_map_hash2node_ptr_.erase(hash);
        m_map_node_ptr2hash_.erase(node_ptr);
        delete node_ptr;

        // TODO:
        // remove edge from m_map_hash_pair2edge_ptr_
    }

    void addEdge(LD edge_attributes, Hash hash_source, Hash hash_destination)
    {
        //std::cout << m_map_hash2node_ptr_.at(hash_source)->getHash() << " " << m_map_hash2node_ptr_.at(hash_destination) << std::endl;
        //std::cout << edge_attributes.src << " " << edge_attributes.dst << std::endl;
        Edge<ND, LD> *edge_ptr = graph.addEdge(
            edge_attributes,
            m_map_hash2node_ptr_.at(hash_source),
            m_map_hash2node_ptr_.at(hash_destination));
        m_map_hash_pair2edge_ptr_.insert({{hash_source, hash_destination}, edge_ptr});
    }

    std::vector<Hash> getHashList() const
    {
        std::vector<Hash> hash_list;
        for (auto kv : m_map_hash2node_ptr_)
        {
            hash_list.push_back(kv.first);
        }
        return hash_list;
    }

    Node<ND, LD> *getNodePtr(const Hash &hash) const
    {
        return m_map_hash2node_ptr_.at(hash);
    }

    Edge<ND, LD> *getEdgePtr(const Hash &hash_source, const Hash &hash_destination) const
    {
        return m_map_hash_pair2edge_ptr_.at({hash_source, hash_destination});
    }

    std::vector<Node<ND, LD> *> getNeighborNodesPtr(const Hash &hash) const
    {
        return m_map_hash2node_ptr_.at(hash).getNeighborNodes();
    }

    std::vector<Hash> getNeighborHashList(const Hash &hash) const
    {
        std::vector<Hash> neighbor_hash_list;
        std::vector<Node<ND, LD> *> neighbor_ptr_list = m_map_hash2node_ptr_.at(hash)->getNeighborNodes();
        for (Node<ND, LD> *neighbor_ptr : neighbor_ptr_list)
        {
            neighbor_hash_list.push_back(m_map_node_ptr2hash_.at(neighbor_ptr));
        }
        return neighbor_hash_list;
    }

    int32_t size()
    {
        return m_map_hash2node_ptr_.size();
    }

    void setupOutputGraph()
    {
        //Setup node position
        std::unordered_map<Hash, int32_t> map_hash2index;

        int32_t node_index = 0;
        m_node_position_list_.clear();
        for (Node<ND, LD> *node_ptr = graph.firstNode; node_ptr; node_ptr = node_ptr->next)
        {
            int32_t x_pos, y_pos;
            node_ptr->data.getPosition(x_pos, y_pos);
            std::vector<int32_t> position{x_pos, y_pos};
            m_node_position_list_.push_back(position);
            map_hash2index.insert({node_ptr->data.getHash(), node_index});
            node_index++;
        }

        m_edge_list_.clear();
        for (Edge<ND, LD> *L = graph.firstEdge; L; L = L->next)
        {
            std::vector<int32_t> edge_info{map_hash2index[L->data.src], map_hash2index[L->data.dst]};
            m_edge_list_.push_back(edge_info);
        }
    }

    std::vector<std::vector<int32_t>> getNodePositions()
    {
        return m_node_position_list_;
    }

    std::vector<std::vector<int32_t>> getEdges()
    {
        return m_edge_list_;
    }

private:
    std::unordered_map<Hash, Node<ND, LD> *> m_map_hash2node_ptr_;
    std::unordered_map<Node<ND, LD> *, Hash> m_map_node_ptr2hash_;
    std::map<std::pair<Hash, Hash>, Edge<ND, LD> *> m_map_hash_pair2edge_ptr_;

    std::vector<std::vector<int32_t>> m_node_position_list_; // For output
    std::vector<std::vector<int32_t>> m_edge_list_;          // For output
};

#endif //PYSKELETON2GRAPH_INCLUDE_GRAPH_HELPER_H_
