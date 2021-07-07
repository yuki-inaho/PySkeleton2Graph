#ifndef PYSKELETON2GRAPH_INCLUDE_LINEAR_CLUSTER_H_
#define PYSKELETON2GRAPH_INCLUDE_LINEAR_CLUSTER_H_

#if __has_include(<Eigen>)
#include <Eigen/Dense>
#include <Eigen/SVD>
#else
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#endif

#include "pixel.h"
#include "graph.h"
#include "graph_helper.h"
#include "typedef_graph.h"

typedef Eigen::Matrix<float, 3, Eigen::Dynamic> Matrix2DPoints;

class LineCoeff
{
public:
    LineCoeff(){};

    void set(const float &p1_x, const float &p1_y, const float &p2_x, const float &p2_y)
    {
        float l_x_tmp = p2_x - p1_x;
        float l_y_tmp = p2_y - p1_y;

        float l_sqrt = std::sqrt(l_x_tmp * l_x_tmp + l_y_tmp * l_y_tmp);
        l_x_tmp /= l_sqrt;
        l_y_tmp /= l_sqrt;

        m_nx_ = -l_y_tmp;
        m_ny_ = l_x_tmp;
        m_constant_ = -(m_nx_ * p1_x + m_ny_ * p1_y);

        /// align vector direction (always upward in the term of 2d image view)
        if (this->l_y() > 0)
        {
            m_nx_ *= -1;
            m_ny_ *= -1;
            m_constant_ *= -1;
        }

        validationNormal();
    }

    void set(const float &n_x_, const float &n_y_, const float &constant_)
    {
        float n_norm = std::sqrt(n_x_ * n_x_ + n_y_ * n_y_);
        m_nx_ = n_x_ / n_norm;
        m_ny_ = n_y_ / n_norm;
        m_constant_ = constant_ / n_norm;

        /// align vector direction
        if (this->l_y() > 0)
        {
            m_nx_ *= -1;
            m_ny_ *= -1;
            m_constant_ *= -1;
        }

        validationNormal();
    }

    float get_fit_error(const float &p_x, const float &p_y)
    {
        return m_nx_ * p_x + m_ny_ * p_y + m_constant_;
    }
    /*
    normal of line model
    */
    float n_x() const
    {
        return m_nx_;
    }

    float n_y() const
    {
        return m_ny_;
    }

    /*
    line direction
    */
    float l_x() const
    {
        return -m_ny_;
    }

    float l_y() const
    {
        return m_nx_;
    }

    float n_constant() const
    {
        return m_constant_;
    }

private:
    void validationNormal()
    {
        float epsilon = 10e-4;
        if (std::abs(std::sqrt(m_nx_ * m_nx_ + m_ny_ * m_ny_) - 1) > epsilon)
        {
            std::cerr << "linear_cluster.h: std::abs(std::sqrt(m_nx_ * m_nx_ + m_ny_ * m_ny_) - 1) > epsilon" << std::endl;
            exit(EXIT_FAILURE);
        }
        else if (this->l_y() > 0)
        {
            std::cerr << "status &= this->l_y() > 0" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    float m_nx_, m_ny_;
    float m_constant_;
};

class LinearCluster
{
public:
    LinearCluster(){};
    LinearCluster(int32_t cluster_label) : m_parent_(this), m_cluster_label_(cluster_label), m_cluster_type_(LinearClusterType::kBridgePointCluster){};
    ~LinearCluster(){};

    int32_t label() const
    {
        return m_cluster_label_;
    }

    void relabel(const int32_t &new_label, SkeletonGraphHelperPtr graph_helper_ptr)
    {
        m_cluster_label_ = new_label;
        for (Hash node_hash : m_node_hash_list_)
        {
            accessByHash(node_hash, graph_helper_ptr)->data.setLabel(new_label);
        }
    }

    int32_t size() const
    {
        return m_node_hash_list_.size();
    }

    LinearClusterType type() const
    {
        return m_cluster_type_;
    }

    /*
    Return fitted line model parameters
    */
    std::vector<float> line_parameter() const
    {
        std::vector<float> parameters{m_line_model_.n_x(), m_line_model_.n_y(), m_line_model_.n_constant()};
        return parameters;
    }

    std::vector<float> line_direction() const
    {
        std::vector<float> parameters{m_line_model_.l_x(), m_line_model_.l_y()};
        return parameters;
    }

    float line_length() const
    {
        return m_line_length_;
    }

    /*
    Return skeleton point positions(px, py)
    */
    std::vector<std::vector<int32_t>> points()
    {
        return m_point_list_;
    }

    /*
    Update intrinsic infromation of each clusters, after addSkeletonPoint() calling
    */
    void update(SkeletonGraphHelperPtr graph_helper_ptr)
    {
        /// Set 2D point (corresponded to a node) information in the cluster
        m_point_list_.clear();
        std::vector<std::shared_ptr<SkeletonGraphNode>> node_ptr_list;
        for (Hash node_hash : m_node_hash_list_)
        {
            int32_t test = accessByHash(node_hash, graph_helper_ptr)->data.getHash();
            int32_t px, py;
            accessByHash(node_hash, graph_helper_ptr)->data.getPosition(px, py);
            std::shared_ptr<SkeletonGraphNode> node_ptr(accessByHash(node_hash, graph_helper_ptr));
            node_ptr_list.push_back(node_ptr);
        }

        for (std::shared_ptr<SkeletonGraphNode> node_ptr : node_ptr_list)
        {
            int32_t px, py;
            node_ptr->data.getPosition(px, py);
            std::vector<int32_t> point{px, py};
            m_point_list_.push_back(point);
        }

        /// Set edge information
        m_map_hash2index_.clear();
        m_edge_list_.clear();
        for (int32_t index = 0; index < node_ptr_list.size(); index++)
        {
            m_map_hash2index_.insert({node_ptr_list[index]->data.getHash(), index});
        }
        for (std::shared_ptr<SkeletonGraphNode> node_ptr : node_ptr_list)
        {
            for (auto edge_ptr = node_ptr->firstOut; edge_ptr; edge_ptr = edge_ptr->nextInFrom)
            {
                if (m_map_hash2index_.count(edge_ptr->data.dst) == 0)
                    continue;
                std::vector<int32_t> edge{m_map_hash2index_.at(edge_ptr->data.src), m_map_hash2index_.at(edge_ptr->data.dst)};
                m_edge_list_.push_back(edge);
            }
        }

        /// Fit 2D points to line model
        fitLine(graph_helper_ptr);
        setEndPointIndices(graph_helper_ptr);
        setJunctionPointIndices(graph_helper_ptr);

        setProjectedPointsToLine();
        setLineLength();
    }

    std::vector<std::vector<int32_t>> edges() const
    {
        return m_edge_list_;
    }

    int32_t hash2index(const Hash &node_hash) const
    {
        return m_map_hash2index_.at(node_hash);
    }

    void addSkeletonPoint(const Hash &node_hash, SkeletonGraphHelperPtr graph_helper_ptr)
    {
        SkeletonGraphNode *node_ptr = accessByHash(node_hash, graph_helper_ptr);
        /// FORME: confirm in the case of kEndPointCluster + kJunctionPointCluster
        if (node_ptr->data.getPointType() == PointType::kEndPoint)
        {
            m_cluster_type_ = LinearClusterType::kEndPointCluster;
        }
        else if (node_ptr->data.getPointType() == PointType::kJunctionPoint)
        {
            m_cluster_type_ = LinearClusterType::kJunctionPointCluster;
        }
        m_node_hash_list_.push_back(node_hash);
    }

    void fitLine(SkeletonGraphHelperPtr graph_helper_ptr)
    {
        if (size() == 1)
        {
            int32_t px, py;
            accessByIndex(0, graph_helper_ptr)->data.getPosition(px, py);
            m_line_model_.set(1.0, 0, px, py);
        }
        else if (size() == 2)
        {
            int32_t p0x, p0y;
            accessByIndex(0, graph_helper_ptr)->data.getPosition(p0x, p0y);
            int32_t p1x, p1y;
            accessByIndex(1, graph_helper_ptr)->data.getPosition(p1x, p1y);
            m_line_model_.set(p0x, p0y, p1x, p1y);
        }
        else
        {
            float n_x, n_y, n_constant;
            fitLineBySVD(n_x, n_y, n_constant, graph_helper_ptr);
            m_line_model_.set(n_x, n_y, n_constant);
        }
    }

    float getFittingError(const float &p_x, const float &p_y)
    {
        return m_line_model_.get_fit_error(p_x, p_y);
    }

    float getMeanSquaredError(SkeletonGraphHelperPtr graph_helper_ptr)
    {
        float sum_error = 0;

        // TODO: Use reduce
        int32_t n_points = m_node_hash_list_.size();
        for (int32_t i = 0; i < n_points; i++)
        {
            int32_t px, py;
            accessByIndex(i, graph_helper_ptr)->data.getPosition(px, py);
            sum_error += getFittingError(px, py);
        }
        return sum_error / n_points;
    }

    std::vector<int32_t> getEndPointIndices() const
    {
        return m_end_point_indices_;
    }

    std::vector<int32_t> getJunctionPointIndices() const
    {
        return m_junction_point_indices_;
    }

    std::vector<SkeletonGraphNode *> getEndPointPtrList(SkeletonGraphHelperPtr graph_helper_ptr) const
    {
        std::vector<SkeletonGraphNode *> node_ptr_list_end_points;
        std::transform(
            m_end_point_indices_.begin(), m_end_point_indices_.end(),
            std::back_inserter(node_ptr_list_end_points),
            [&](const Hash &point_index)
            {
                return accessByIndex(point_index, graph_helper_ptr);
            });
        return node_ptr_list_end_points;
    }

    std::vector<SkeletonGraphNode *> getJunctionPointPtrList(SkeletonGraphHelperPtr graph_helper_ptr) const
    {
        std::vector<SkeletonGraphNode *> node_ptr_list_junction_points;
        std::transform(
            m_junction_point_indices_.begin(), m_junction_point_indices_.end(),
            std::back_inserter(node_ptr_list_junction_points),
            [&](const Hash &point_index)
            {
                return accessByIndex(point_index, graph_helper_ptr);
            });
        return node_ptr_list_junction_points;
    }

private:
    inline SkeletonGraphNode *accessByIndex(const int32_t &node_index, SkeletonGraphHelperPtr graph_helper_ptr) const
    {
        return graph_helper_ptr->getNodePtr(m_node_hash_list_[node_index]);
    }

    inline SkeletonGraphNode *accessByHash(const Hash &node_hash, SkeletonGraphHelperPtr graph_helper_ptr) const
    {
        return graph_helper_ptr->getNodePtr(node_hash);
    }

    void normal(float &n_x, float &n_y)
    {
        n_x = m_line_model_.n_x();
        n_y = m_line_model_.n_y();
    }

    void fitLineBySVD(float &n_x_output, float &n_y_output, float &constant_output, SkeletonGraphHelperPtr graph_helper_ptr)
    {
        Matrix2DPoints mat_points;
        mat_points.resize(3, m_node_hash_list_.size());
        int32_t n_points = m_node_hash_list_.size();
        for (int32_t i = 0; i < n_points; i++)
        {
            int32_t px, py;
            accessByIndex(i, graph_helper_ptr)->data.getPosition(px, py);
            mat_points(0, i) = px;
            mat_points(1, i) = py;
            mat_points(2, i) = 1.0;
        }
        Eigen::JacobiSVD<Eigen::Matrix<float, 3, Eigen::Dynamic>> svd(mat_points, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix<float, 3, 3> u = svd.matrixU();
        n_x_output = u(0, 2);
        n_y_output = u(1, 2);
        constant_output = u(2, 2);
    }

    bool isEndPoint(const Hash &node_hash, SkeletonGraphHelperPtr graph_helper_ptr) const
    {
        SkeletonGraphNode *node_ptr = accessByHash(node_hash, graph_helper_ptr);
        if (node_ptr->data.getPointType() == PointType::kEndPoint)
            return true;

        int32_t num_edge = 0;
        for (auto edge_ptr = node_ptr->firstOut; edge_ptr; edge_ptr = edge_ptr->nextInFrom)
        {
            int32_t label_dst = accessByHash(edge_ptr->data.dst, graph_helper_ptr)->data.getLabel();
            if (m_cluster_label_ != label_dst)
                return true;
            num_edge++;
        }
        if (num_edge < 2)
        {
            return true;
        }
        return false;
    }

    /*
    Store end points in the sense of linear cluster
    */
    void setEndPointIndices(SkeletonGraphHelperPtr graph_helper_ptr)
    {
        m_end_point_indices_.clear();
        int32_t index = 0;
        for (Hash node_hash : m_node_hash_list_)
        {
            if (isEndPoint(node_hash, graph_helper_ptr))
                m_end_point_indices_.push_back(index);
            index++;
        }
    }

    /*
    Store PointType::kJunctionPoint points
    */
    void setJunctionPointIndices(SkeletonGraphHelperPtr graph_helper_ptr)
    {
        m_junction_point_indices_.clear();
        int32_t index = 0;
        for (Hash node_hash : m_node_hash_list_)
        {
            if (accessByHash(node_hash, graph_helper_ptr)->data.getPointType() == PointType::kJunctionPoint)
                m_junction_point_indices_.push_back(index);
            index++;
        }
    }

    std::vector<int32_t> projectionPointToLine(const std::vector<int32_t> &point)
    {
        float p_x = static_cast<float>(point[0]);
        float p_y = static_cast<float>(point[1]);
        float distance_to_line = p_x * m_line_model_.n_x() + p_y * m_line_model_.n_y() + m_line_model_.n_constant();
        int32_t p_x_projected = static_cast<int32_t>(p_x - distance_to_line * m_line_model_.n_x());
        int32_t p_y_projected = static_cast<int32_t>(p_y - distance_to_line * m_line_model_.n_y());
        std::vector<int32_t> projected_point{p_x_projected, p_y_projected};
        return projected_point;
    }

    void setProjectedPointsToLine()
    {
        m_projected_point_list_.clear();
        std::transform(
            m_point_list_.begin(), m_point_list_.end(),
            std::back_inserter(m_projected_point_list_),
            [&](const std::vector<int32_t> point)
            { return projectionPointToLine(point); });
    }

    std::vector<int32_t> getPointsValueX(const std::vector<std::vector<int32_t>> &point_list)
    {
        std::vector<int32_t> projected_point_x_list;
        std::transform(
            point_list.begin(), point_list.end(),
            std::back_inserter(projected_point_x_list),
            [&](const std::vector<int32_t> point)
            { return point[0]; });
        return projected_point_x_list;
    }

    std::vector<int32_t> getPointsValueY(const std::vector<std::vector<int32_t>> &point_list)
    {
        std::vector<int32_t> projected_point_y_list;
        std::transform(
            point_list.begin(), point_list.end(),
            std::back_inserter(projected_point_y_list),
            [&](const std::vector<int32_t> point)
            { return point[1]; });
        return projected_point_y_list;
    }

    void setLineLength()
    {
        std::vector<int32_t> projected_point_y_list = getPointsValueY(m_projected_point_list_);
        std::vector<int32_t>::iterator min_it = std::min_element(projected_point_y_list.begin(), projected_point_y_list.end());
        std::vector<int32_t>::iterator max_it = std::max_element(projected_point_y_list.begin(), projected_point_y_list.end());
        m_index_argmin_p_y_ = std::distance(projected_point_y_list.begin(), min_it);
        m_index_argmax_p_y_ = std::distance(projected_point_y_list.begin(), max_it);
        float diff_x = m_projected_point_list_[m_index_argmax_p_y_][0] - m_projected_point_list_[m_index_argmin_p_y_][0];
        float diff_y = m_projected_point_list_[m_index_argmax_p_y_][1] - m_projected_point_list_[m_index_argmin_p_y_][1];
        m_line_length_ = std::sqrt(diff_x * diff_x + diff_y * diff_y);
    }

    LinearClusterType m_cluster_type_;
    int32_t m_cluster_label_;
    float m_cluster_proximity_threshold_;

    float m_line_length_;
    std::unordered_map<Hash, int32_t> m_map_hash2index_;
    std::vector<std::vector<int32_t>> m_point_list_;
    std::vector<std::vector<int32_t>> m_projected_point_list_;
    std::vector<Hash> m_node_hash_list_;
    std::vector<std::vector<int32_t>> m_edge_list_;
    std::vector<int32_t> m_end_point_indices_;
    std::vector<int32_t> m_junction_point_indices_;
    LinearCluster *m_parent_;
    LineCoeff m_line_model_;

    int32_t m_index_argmin_p_y_, m_index_argmax_p_y_; //mainly for debug
};

/*
Return connectivity relation between two clusters
*/
// TODO: write more simply
void identificateClusterConnection(const std::vector<LinearCluster> &linear_cluster_list, SkeletonGraphHelperPtr graph_helper_ptr, std::vector<std::vector<int32_t>> &index_pairs_mutual_clusters, std::vector<std::vector<Hash>> &point_index_pairs_mutual_clusters)
{
    index_pairs_mutual_clusters.clear();
    point_index_pairs_mutual_clusters.clear();

    // Set searching target points in each cluster
    int32_t n_clusters = linear_cluster_list.size();
    std::vector<std::vector<SkeletonGraphNode *>> search_point_ptr_list_each_cluster;
    for (int32_t index_cluster = 0; index_cluster < n_clusters; index_cluster++)
    {
        std::vector<SkeletonGraphNode *> search_point_ptrs;
        std::vector<SkeletonGraphNode *> end_point_ptr_list = linear_cluster_list[index_cluster].getEndPointPtrList(graph_helper_ptr);
        std::vector<SkeletonGraphNode *> junction_point_ptr_list = linear_cluster_list[index_cluster].getJunctionPointPtrList(graph_helper_ptr);
        std::copy(end_point_ptr_list.begin(), end_point_ptr_list.end(), std::back_inserter(search_point_ptrs));
        std::copy(junction_point_ptr_list.begin(), junction_point_ptr_list.end(), std::back_inserter(search_point_ptrs));
        search_point_ptr_list_each_cluster.push_back(search_point_ptrs);
    }

    // Search cluster connection
    for (int32_t index_cluster = 0; index_cluster < n_clusters; index_cluster++)
    {
        std::vector<SkeletonGraphNode *> search_point_ptr_list = search_point_ptr_list_each_cluster[index_cluster];
        for (int32_t index_cluster_compare = 0; index_cluster_compare < n_clusters; index_cluster_compare++)
        {
            if (index_cluster <= index_cluster_compare)
                continue;
            std::vector<SkeletonGraphNode *> search_point_ptr_list_compare = search_point_ptr_list_each_cluster[index_cluster_compare];

            // check connectivity between two end points
            bool exists_edge = false;
            Hash search_point_hash, search_point_hash_compare;
            for (SkeletonGraphNode *search_point_ptr : search_point_ptr_list)
            {
                for (SkeletonGraphNode *search_point_ptr_compare : search_point_ptr_list_compare)
                {
                    search_point_hash = search_point_ptr->data.getHash();
                    search_point_hash_compare = search_point_ptr_compare->data.getHash();
                    exists_edge |= graph_helper_ptr->existsEdge(search_point_hash, search_point_hash_compare);
                    if (exists_edge)
                        break;
                }
                if (exists_edge)
                    break;
            }
            if (exists_edge)
            {
                /// Actual mutual connectivity is undirectional, but for simplify, store unique combination of index pair will be stored.
                std::vector<int32_t> cluster_index_pair{index_cluster, index_cluster_compare};
                index_pairs_mutual_clusters.push_back(cluster_index_pair);

                int32_t search_point_index = linear_cluster_list[index_cluster].hash2index(search_point_hash);
                int32_t search_point_index_compare = linear_cluster_list[index_cluster_compare].hash2index(search_point_hash_compare);
                std::vector<int32_t> node_index_pair{search_point_index, search_point_index_compare};
                point_index_pairs_mutual_clusters.push_back(node_index_pair);
            }
        }
    }
}

#endif // PYSKELETON2GRAPH_INCLUDE_LINEAR_CLUSTER_H_