#ifndef PYSKELETON2GRAPH_INCLUDE_GRAPH_HELPER_H_
#define PYSKELETON2GRAPH_INCLUDE_GRAPH_HELPER_H_

#include <map>

#include "graph.h"
#include "typedef_pixel.h"

// TODO: use smart ptr
template <typename ND, typename LD>
class GraphHelper {
   public:
    Graph<ND, LD> &graph;

    GraphHelper(){};
    GraphHelper(Graph<ND, LD> &graph) : graph(graph){};
    void addNode(Hash hash, ND data) {
        NodePtr<ND, LD> node_ptr = graph.addNode(data.clone());
        m_map_hash_to_node_ptr_.insert(std::pair<Hash, NodePtr<ND, LD>>(hash, node_ptr));
        m_map_node_ptr_to_hash_.insert(std::pair<NodePtr<ND, LD>, Hash>(node_ptr, hash));
    }

    bool hasNode(const Hash &hash) const { return m_map_hash_to_node_ptr_.find(hash) != m_map_hash_to_node_ptr_.end(); }

    /*
        Edge removal connected with removed nodes processes doesn't conduct on below function.
        It is necessary to call "refreshGraphInfo()" after calling the function.

        Assume graph is undirectional
    */
    void removeNode(const Hash &hash) {
        std::vector<Hash> hash_list_neighbor = getNeighborHashList(hash);
        if (hash_list_neighbor.size() > 1) {
            for (Hash hash_source : hash_list_neighbor) {
                for (Hash hash_destination : hash_list_neighbor) {
                    if (hash_source == hash_destination) continue;
                    EdgeSkeletonPixels edge_attributes =
                        EdgeSkeletonPixels(m_map_hash_to_node_ptr_.at(hash_source)->data, m_map_hash_to_node_ptr_.at(hash_destination)->data);
                    addEdge(edge_attributes, hash_source, hash_destination);
                }
            }
        }

        for (Hash hash_neighbor : hash_list_neighbor) {
            removeEdge(hash, hash_neighbor);
            removeEdge(hash_neighbor, hash);
        }

        NodePtr<ND, LD> node_ptr = m_map_hash_to_node_ptr_.at(hash);
        m_map_hash_to_node_ptr_.erase(hash);
        m_map_node_ptr_to_hash_.erase(node_ptr);
        delete node_ptr;
    }

    void addEdge(LD edge_attributes, Hash hash_source, Hash hash_destination) {
        EdgePtr<ND, LD> edge_ptr =
            graph.addEdge(edge_attributes, m_map_hash_to_node_ptr_.at(hash_source), m_map_hash_to_node_ptr_.at(hash_destination));
        m_map_hash_pair_to_edge_ptr_.insert({{hash_source, hash_destination}, edge_ptr});
    }

    void removeEdge(Hash hash_source, Hash hash_destination) {
        delete m_map_hash_pair_to_edge_ptr_.at({hash_source, hash_destination});
        m_map_hash_pair_to_edge_ptr_.erase({hash_source, hash_destination});
    }

    /*
    hash_node_set includes hash_core_node
    */
    void compressNodeSet(const Hash &hash_core_node, const std::vector<Hash> &hash_node_set) {
        // Get peripheral node
        std::vector<Hash> hash_list_peripheral_node;
        for (Hash hash_node : hash_node_set) {
            std::vector<Hash> hash_list_neighbor = getNeighborHashList(hash_node);
            std::copy_if(hash_list_neighbor.begin(), hash_list_neighbor.end(), std::back_inserter(hash_list_peripheral_node),
                         [=](Hash hash) { return *std::find(hash_node_set.begin(), hash_node_set.end(), hash) != hash; });
        }

        for (Hash hash_node : hash_node_set) {
            if (!hasNode(hash_node)) continue;
            NodePtr<ND, LD> node_ptr = m_map_hash_to_node_ptr_.at(hash_node);
            std::vector<NodePtr<ND, LD>> neighbor_node_ptr_list = getNeighborNodesPtr(hash_node);
            for (NodePtr<ND, LD> neighbor_node_ptr : neighbor_node_ptr_list) {
                removeEdge(node_ptr->data.getHash(), neighbor_node_ptr->data.getHash());
                removeEdge(neighbor_node_ptr->data.getHash(), node_ptr->data.getHash());
            }
            if (hash_node != hash_core_node) {
                m_map_hash_to_node_ptr_.erase(hash_node);
                m_map_node_ptr_to_hash_.erase(node_ptr);
                delete node_ptr;
            }
        }

        for (Hash hash_peripheral_node : hash_list_peripheral_node) {
            EdgeSkeletonPixels edge_attributes_forward =
                EdgeSkeletonPixels(m_map_hash_to_node_ptr_.at(hash_core_node)->data, m_map_hash_to_node_ptr_.at(hash_peripheral_node)->data);
            EdgeSkeletonPixels edge_attributes_backward =
                EdgeSkeletonPixels(m_map_hash_to_node_ptr_.at(hash_peripheral_node)->data, m_map_hash_to_node_ptr_.at(hash_core_node)->data);
            addEdge(edge_attributes_forward, hash_core_node, hash_peripheral_node);
            addEdge(edge_attributes_backward, hash_peripheral_node, hash_core_node);
        }
    }

    std::vector<Hash> getHashList() const {
        std::vector<Hash> hash_list;
        for (auto kv : m_map_hash_to_node_ptr_) {
            hash_list.push_back(kv.first);
        }
        return hash_list;
    }

    NodePtr<ND, LD> getNodePtr(const Hash &hash) const { return m_map_hash_to_node_ptr_.at(hash); }

    EdgePtr<ND, LD> getEdgePtr(const Hash &hash_source, const Hash &hash_destination) const {
        return m_map_hash_pair_to_edge_ptr_.at({hash_source, hash_destination});
    }

    std::vector<NodePtr<ND, LD>> getNeighborNodesPtr(const Hash &hash) const { return m_map_hash_to_node_ptr_.at(hash)->getNeighborNodes(); }

    std::vector<Hash> getNeighborHashList(const Hash &hash) const {
        std::vector<Hash> neighbor_hash_list;
        if (!hasNode(hash)) return neighbor_hash_list;
        std::vector<NodePtr<ND, LD>> neighbor_ptr_list = m_map_hash_to_node_ptr_.at(hash)->getNeighborNodes();
        for (NodePtr<ND, LD> neighbor_ptr : neighbor_ptr_list) {
            neighbor_hash_list.push_back(m_map_node_ptr_to_hash_.at(neighbor_ptr));
        }
        return neighbor_hash_list;
    }

    int32_t size() { return m_map_hash_to_node_ptr_.size(); }

    void validateNode() {
        bool flag_skip_validation = false;
        while (!flag_skip_validation) {
            flag_skip_validation = true;
            for (NodePtr<ND, LD> node_ptr = graph.firstNode; node_ptr; node_ptr = node_ptr->next) {
                // Update node information
                int8_t connectivity = calcNodeConnectivity(node_ptr);
                if (connectivity == 0) {
                    removeNode(node_ptr->data.getHash());
                    flag_skip_validation = false;
                }
            }
        }
    }

    void refreshGraphInfo() {
        validateNode();
        m_map_hash_to_node_ptr_.clear();
        m_map_node_ptr_to_hash_.clear();
        m_map_hash_pair_to_edge_ptr_.clear();
        for (NodePtr<ND, LD> node_ptr = graph.firstNode; node_ptr; node_ptr = node_ptr->next) {
            // Update node information
            node_ptr->data.setConnectivity(calcNodeConnectivity(node_ptr));
            node_ptr->data.setPointType();

            Hash hash = node_ptr->data.getHash();
            m_map_hash_to_node_ptr_.insert(std::pair<Hash, NodePtr<ND, LD>>(hash, node_ptr));
            m_map_node_ptr_to_hash_.insert(std::pair<NodePtr<ND, LD>, Hash>(node_ptr, hash));
        }

        for (EdgePtr<ND, LD> edge_ptr = graph.firstEdge; edge_ptr; edge_ptr = edge_ptr->next) {
            // Update node information
            edge_ptr->data.resetPixelPair(edge_ptr->from->data, edge_ptr->to->data);
            m_map_hash_pair_to_edge_ptr_.insert({{edge_ptr->data.src, edge_ptr->data.dst}, edge_ptr});
        }
    }

    void validateGraphInfo() {
        if (graph.m_num_nodes_ != m_map_node_ptr_to_hash_.size()) {
            std::cerr << "graph.m_num_nodes_ != m_map_node_ptr_to_hash_.size()" << std::endl;
            exit(EXIT_FAILURE);
        }

        if (graph.m_num_edges_ != m_map_hash_pair_to_edge_ptr_.size()) {
            std::cerr << "graph.m_num_edges_ != m_map_hash_pair_to_edge_ptr_.size(): " << graph.m_num_edges_ << " "
                      << m_map_hash_pair_to_edge_ptr_.size() << std::endl;
            exit(EXIT_FAILURE);
        }

        std::vector<Hash> hash_list;
        for (auto kv : m_map_hash_to_node_ptr_) {
            hash_list.push_back(kv.first);
        }
        for (Hash node_hash : hash_list) {
            if (node_hash != getNodePtr(node_hash)->data.getHash()) {
                std::cerr << "node_hash != getNodePtr(node_hash)->data.getHash():" << node_hash << "," << getNodePtr(node_hash)->data.getHash()
                          << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        for (NodePtr<ND, LD> node_ptr = graph.firstNode; node_ptr; node_ptr = node_ptr->next) {
            std::vector<NodePtr<ND, LD>> neighbor_ptr_list = node_ptr->getNeighborNodes();
            for (NodePtr<ND, LD> neighbor_ptr : neighbor_ptr_list) {
                Hash hash_source = node_ptr->data.getHash();
                Hash hash_destination = neighbor_ptr->data.getHash();
                if (m_map_hash_pair_to_edge_ptr_.count({hash_source, hash_destination}) == 0 ||
                    m_map_hash_pair_to_edge_ptr_.count({hash_destination, hash_source}) == 0) {
                    std::cerr << "Edge information is invalid: (source:" << hash_source << "), (target:" << hash_destination << ")" << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
        }
    }

    void setupOutputGraph() {
        // Setup node position
        std::unordered_map<Hash, int32_t> map_hash2index;
        int32_t node_index = 0;
        m_node_position_list_.clear();
        for (NodePtr<ND, LD> node_ptr = graph.firstNode; node_ptr; node_ptr = node_ptr->next) {
            int32_t x_pos, y_pos;
            node_ptr->data.getPosition(x_pos, y_pos);
            std::vector<int32_t> position{x_pos, y_pos};
            m_node_position_list_.push_back(position);
            map_hash2index.insert({node_ptr->data.getHash(), node_index});
            node_index++;
        }

        // Reset Labels
        if (m_node_label_list_.size() == 0) {
            m_node_label_list_.clear();
            m_node_label_list_.resize(m_node_position_list_.size());
            std::fill(m_node_label_list_.begin(), m_node_label_list_.end(), 0);
        }

        // Reset Edge information
        m_edge_list_.clear();
        for (EdgePtr<ND, LD> edge_ptr = graph.firstEdge; edge_ptr; edge_ptr = edge_ptr->next) {
            std::vector<int32_t> edge_info{map_hash2index[edge_ptr->data.src], map_hash2index[edge_ptr->data.dst]};
            m_edge_list_.push_back(edge_info);
        }
    }

    std::vector<int32_t> getNodeLabels() { return m_node_label_list_; }

    std::vector<std::vector<int32_t>> getNodePositions() { return m_node_position_list_; }

    std::vector<std::vector<int32_t>> getEdges() { return m_edge_list_; }

    bool existsEdge(const int32_t &node1_hash, const int32_t &node2_hash) const {
        return m_map_hash_pair_to_edge_ptr_.count(std::pair<Hash, Hash>{node1_hash, node2_hash}) > 0;
    }

    void setConnectedComponentLabels(const std::vector<std::vector<Hash>> &hash_list_each_cc) {
        m_node_label_list_.clear();
        std::unordered_map<Hash, int32_t> map_hash2index;
        int32_t node_index = 0;
        for (NodePtr<ND, LD> node_ptr = graph.firstNode; node_ptr; node_ptr = node_ptr->next) {
            map_hash2index.insert({node_ptr->data.getHash(), node_index});
            node_index++;
        }
        m_node_label_list_.resize(map_hash2index.size());

        int32_t label_cc = 1;
        for (std::vector<Hash> hash_list_cc : hash_list_each_cc) {
            for (Hash hash_cc_elem : hash_list_cc) {
                m_node_label_list_[map_hash2index[hash_cc_elem]] = label_cc;
                getNodePtr(hash_cc_elem)->data.setLabel(label_cc);
            }
            label_cc++;
        }
    }

    void updateOutputNodeLabels() {
        m_node_label_list_.clear();
        std::unordered_map<Hash, int32_t> map_hash2index;
        int32_t node_index = 0;
        for (Node<ND, LD> *node_ptr = graph.firstNode; node_ptr; node_ptr = node_ptr->next) {
            map_hash2index.insert({node_ptr->data.getHash(), node_index});
            node_index++;
        }
        m_node_label_list_.resize(map_hash2index.size());

        for (Node<ND, LD> *node_ptr = graph.firstNode; node_ptr; node_ptr = node_ptr->next) {
            m_node_label_list_[map_hash2index[node_ptr->data.getHash()]] = node_ptr->data.getLabel();
        }
    }

   private:
    int8_t calcNodeConnectivity(const NodePtr<ND, LD> node_ptr) {
        int8_t connectivity = 0;
        for (EdgePtr<ND, LD> x = node_ptr->firstOut; x; x = x->nextInFrom) {
            connectivity++;
        }
        return connectivity;
    }

    std::unordered_map<Hash, NodePtr<ND, LD>> m_map_hash_to_node_ptr_;
    std::unordered_map<NodePtr<ND, LD>, Hash> m_map_node_ptr_to_hash_;
    std::map<std::pair<Hash, Hash>, EdgePtr<ND, LD>> m_map_hash_pair_to_edge_ptr_;

    // For output
    std::vector<int32_t> m_node_label_list_;
    std::vector<std::vector<int32_t>> m_node_position_list_;
    std::vector<std::vector<int32_t>> m_edge_list_;
};

#endif  // PYSKELETON2GRAPH_INCLUDE_GRAPH_HELPER_H_
