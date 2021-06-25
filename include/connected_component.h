#ifndef PYSKELETON2GRAPH_INCLUDE_CONNECTED_COMPONENT_H_
#define PYSKELETON2GRAPH_INCLUDE_CONNECTED_COMPONENT_H_

#include <algorithm>
#include <iostream>
#include "enum.h"
#include "pixel.h"
#include "graph.h"
#include "graph_helper.h"

typedef Node<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraphNode;
typedef Edge<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraphEdge;
typedef GraphHelper<SkeletonPixel, EdgeSkeletonPixels> *SkeletonGraphHelperPtr;

class GraphConnectedComponent
{
public:
    GraphConnectedComponent(){};
    GraphConnectedComponent(const SkeletonGraphHelperPtr graph_helper_ptr) : m_graph_helper_ptr_(graph_helper_ptr){};
    void setup()
    {
        setPointInformation();
        std::vector<Hash> hash_list = m_graph_helper_ptr_->getHashList();
        int node_count = 0;
        for (auto it = hash_list.begin(); it != hash_list.end(); ++it)
        {
            m_map_hash2index_.insert(std::pair<Hash, int32_t>{*it, node_count});
            node_count++;
        }
    }

    void compute(const ConnectedComponent &algorithm_type = ConnectedComponent::kSimple, const float &directional_threshold = 50)
    {
        int32_t n_node = m_graph_helper_ptr_->size();
        std::vector<bool> is_visited(n_node, false);
        // TODO: reduce line
        for (Hash hash_end_point : m_hash_list_end_points_)
        {
            std::vector<Hash> connected_component_list;
            if (is_visited[hash2index(hash_end_point)])
                continue;
            if (algorithm_type == ConnectedComponent::kSimple)
            {
                search_connected_component_simple(hash_end_point, -1, is_visited, connected_component_list);
            }
            else
            {
                search_connected_component_directional(hash_end_point, -1, is_visited, connected_component_list, directional_threshold);
            }
            m_connected_component_list_.push_back(connected_component_list);
        }
        for (Hash hash_rest_point : m_hash_list_whole_points_)
        {
            std::vector<Hash> connected_component_list;
            if (is_visited[hash2index(hash_rest_point)])
                continue;
            if (algorithm_type == ConnectedComponent::kSimple)
            {
                search_connected_component_simple(hash_rest_point, -1, is_visited, connected_component_list);
            }
            else
            {
                search_connected_component_directional(hash_rest_point, -1, is_visited, connected_component_list, directional_threshold);
            }
            m_connected_component_list_.push_back(connected_component_list);
        }
    }

    std::vector<std::vector<Hash>> getConnectedComponent()
    {
        return m_connected_component_list_;
    }

private:
    int32_t hash2index(Hash hash)
    {
        return m_map_hash2index_.at(hash);
    }

    void search_connected_component_simple(const Hash &hash_v, const Hash &hash_parent, std::vector<bool> &is_visited, std::vector<Hash> &connected_component_list)
    {
        if (is_visited[hash2index(hash_v)])
            return;

        is_visited[hash2index(hash_v)] = true;
        connected_component_list.push_back(hash_v);
        std::vector<Hash> neighbor_hash_list_v = m_graph_helper_ptr_->getNeighborHashList(hash_v);
        for (Hash hash_n : neighbor_hash_list_v)
        {
            if (hash_n != hash_parent)
                search_connected_component_simple(hash_n, hash_v, is_visited, connected_component_list);
        }
    }

    /*
        unit is degree
    */
    float get_edge_directional_difference(const Hash &hash_ancester, const Hash &hash_source, const Hash &hash_target)
    {
        float edge_angle_from = m_graph_helper_ptr_->getEdgePtr(hash_ancester, hash_source)->data.getEdgeAngle();
        float edge_angle_to = m_graph_helper_ptr_->getEdgePtr(hash_source, hash_target)->data.getEdgeAngle();
        return std::abs(edge_angle_to - edge_angle_from) / M_PI * 180.0;
    }

    bool check_directional_condition(const Hash &hash_ancester, const Hash &hash_source, const Hash &hash_target, const float &directional_threshold)
    {
        if (hash_ancester == -1)
            return true;
        return get_edge_directional_difference(hash_ancester, hash_source, hash_target) < directional_threshold;
    }

    void search_connected_component_directional(const Hash &hash_v, const Hash &hash_parent, std::vector<bool> &is_visited, std::vector<Hash> &connected_component_list, const float &directional_threshold)
    {
        if (is_visited[hash2index(hash_v)])
            return;
        is_visited[hash2index(hash_v)] = true;
        connected_component_list.push_back(hash_v);
        std::vector<Hash> neighbor_hash_list_v = m_graph_helper_ptr_->getNeighborHashList(hash_v);

        Hash hash_next_node;
        std::vector<Hash> hash_list_next_node_candidate;
        std::copy_if(
            neighbor_hash_list_v.begin(), neighbor_hash_list_v.end(),
            std::back_inserter(hash_list_next_node_candidate),
            [&](Hash hash_neighbor)
            {
                return hash_neighbor != hash_parent;
            });

        if (hash_list_next_node_candidate.size() == 0)
        {
            return;
        }
        else if (hash_list_next_node_candidate.size() == 1)
        {
            hash_next_node = hash_list_next_node_candidate[0];
            if (!check_directional_condition(hash_parent, hash_v, hash_next_node, directional_threshold))
                return;
            search_connected_component_directional(hash_next_node, hash_v, is_visited, connected_component_list, directional_threshold);
        }
        else
        {
            if (hash_parent != -1)
            {
                std::vector<float> edge_directional_difference_list;
                for (Hash hash_candidate : hash_list_next_node_candidate)
                {
                    float edge_directional_difference = get_edge_directional_difference(hash_parent, hash_v, hash_candidate);
                    edge_directional_difference_list.push_back(edge_directional_difference);
                }
                std::vector<float>::iterator max_iter = std::max_element(edge_directional_difference_list.begin(), edge_directional_difference_list.end());
                int32_t argmax_hash_index = std::distance(edge_directional_difference_list.begin(), max_iter);
                hash_next_node = hash_list_next_node_candidate[argmax_hash_index];
                if (!check_directional_condition(hash_parent, hash_v, hash_next_node, directional_threshold))
                    return;
                search_connected_component_directional(hash_next_node, hash_v, is_visited, connected_component_list, directional_threshold);
            }
            else
            {
                for (Hash hash_next_node_ : hash_list_next_node_candidate)
                    search_connected_component_directional(hash_next_node_, hash_v, is_visited, connected_component_list, directional_threshold);
            }
        }
        return;
        /*
        for (Hash hash_n : neighbor_hash_list_v)
        {
            if (hash_n != hash_parent)
            {
                if (!check_directional_condition(hash_parent, hash_v, hash_n, directional_threshold))
                    continue;
                search_connected_component_directional(hash_n, hash_v, is_visited, connected_component_list, directional_threshold);
            }
        }
        */
    }

    void setPointInformation()
    {
        m_hash_list_end_points_.clear();
        m_hash_list_whole_points_.clear();
        std::vector<Hash> hash_list_temp = m_graph_helper_ptr_->getHashList();
        std::copy_if(
            hash_list_temp.begin(), hash_list_temp.end(),
            std::back_inserter(m_hash_list_end_points_),
            [&](Hash hash)
            {
                return m_graph_helper_ptr_->getNodePtr(hash)->data.getPointType() == PointType::kEndPoint;
            });
        std::copy(
            hash_list_temp.begin(), hash_list_temp.end(),
            std::back_inserter(m_hash_list_whole_points_));
    }

    std::unordered_map<Hash, int32_t> m_map_hash2index_;
    SkeletonGraphHelperPtr m_graph_helper_ptr_;
    std::vector<Hash> m_hash_list_end_points_;
    std::vector<Hash> m_hash_list_whole_points_;
    std::vector<std::vector<Hash>> m_connected_component_list_;
};

#endif // PYSKELETON2GRAPH_INCLUDE_CONNECTED_COMPONENT_H_
