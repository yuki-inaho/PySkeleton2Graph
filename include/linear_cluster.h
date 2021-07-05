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
        float l_x = p2_x - p1_x;
        float l_y = p2_y - p1_y;

        float l_sqrt = std::sqrt(l_x * l_x + l_y * l_y);
        l_x /= l_sqrt;
        l_y /= l_sqrt;

        m_nx_ = -l_y;
        m_ny_ = l_x;

        /// align vector direction
        if (m_ny_ > 0)
        {
            m_nx_ *= -1;
            m_ny_ *= -1;
        }
        m_constant_ = -(m_nx_ * p1_x + m_ny_ * p1_y);
    }

    void set(const float &n_x_, const float &n_y_, const float &constant_)
    {
        float n_norm = std::sqrt(n_x_ * n_x_ + n_y_ * n_y_);
        m_nx_ = n_x_ / n_norm;
        m_ny_ = n_y_ / n_norm;

        /// align vector direction
        if (m_ny_ > 0)
        {
            m_nx_ *= -1;
            m_ny_ *= -1;
        }
        m_constant_ = constant_ / n_norm;
    }

    float get_fit_error(const float &p_x, const float &p_y)
    {
        return m_nx_ * p_x + m_ny_ * p_y + m_constant_;
    }

    float n_x() const
    {
        return m_nx_;
    }

    float n_y() const
    {
        return m_ny_;
    }

    float n_constant() const
    {
        return m_constant_;
    }

private:
    float m_nx_;
    float m_ny_;
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
            graph_helper_ptr->getNodePtr(node_hash)->data.setLabel(new_label);
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
    std::vector<float> line() const
    {
        std::vector<float> parameters{m_line_model_.n_x(), m_line_model_.n_y(), m_line_model_.n_constant()};
        return parameters;
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
            int32_t test = graph_helper_ptr->getNodePtr(node_hash)->data.getHash();
            int32_t px, py;
            graph_helper_ptr->getNodePtr(node_hash)->data.getPosition(px, py);
            std::shared_ptr<SkeletonGraphNode> node_ptr(graph_helper_ptr->getNodePtr(node_hash));
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
    }

    std::vector<std::vector<int32_t>> edges() const
    {
        return m_edge_list_;
    }

    void addSkeletonPoint(const Hash &hash, SkeletonGraphHelperPtr graph_helper_ptr)
    {
        SkeletonGraphNode *node_ptr = graph_helper_ptr->getNodePtr(hash);
        /// FORME: confirm in the case of kEndPointCluster + kJunctionPointCluster
        if (node_ptr->data.getPointType() == PointType::kEndPoint)
        {
            m_cluster_type_ = LinearClusterType::kEndPointCluster;
        }
        else if (node_ptr->data.getPointType() == PointType::kJunctionPoint)
        {
            m_cluster_type_ = LinearClusterType::kJunctionPointCluster;
        }
        m_node_hash_list_.push_back(hash);
    }

    void fitLine(SkeletonGraphHelperPtr graph_helper_ptr)
    {
        if (size() == 1)
        {
            int32_t px, py;
            graph_helper_ptr->getNodePtr(m_node_hash_list_[0])->data.getPosition(px, py);
            m_line_model_.set(1.0, 0, px, py);
        }
        else if (size() == 2)
        {
            int32_t p0x, p0y;
            graph_helper_ptr->getNodePtr(m_node_hash_list_[0])->data.getPosition(p0x, p0y);
            int32_t p1x, p1y;
            graph_helper_ptr->getNodePtr(m_node_hash_list_[1])->data.getPosition(p1x, p1y);
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
            graph_helper_ptr->getNodePtr(m_node_hash_list_[i])->data.getPosition(px, py);
            sum_error += getFittingError(px, py);
        }
        return sum_error / n_points;
    }

    std::vector<int32_t> getEndPointIndices() const
    {
        return m_end_point_indices_;
    }

private:
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
            graph_helper_ptr->getNodePtr(m_node_hash_list_[i])->data.getPosition(px, py);
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

    bool isEndPoint(const Hash &hash, SkeletonGraphHelperPtr graph_helper_ptr) const
    {
        SkeletonGraphNode *node_ptr = graph_helper_ptr->getNodePtr(hash);
        if (node_ptr->data.getPointType() == PointType::kEndPoint)
            return true;

        int32_t num_edge = 0;
        for (auto edge_ptr = node_ptr->firstOut; edge_ptr; edge_ptr = edge_ptr->nextInFrom)
        {
            int32_t label_dst = graph_helper_ptr->getNodePtr(edge_ptr->data.dst)->data.getLabel();
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

    LinearClusterType m_cluster_type_;
    int32_t m_cluster_label_;
    float m_cluster_proximity_threshold_;

    std::unordered_map<Hash, int32_t> m_map_hash2index_;
    std::vector<std::vector<int32_t>> m_point_list_;
    std::vector<Hash> m_node_hash_list_;
    std::vector<std::vector<int32_t>> m_edge_list_;
    std::vector<int32_t> m_end_point_indices_;
    LinearCluster *m_parent_;
    LineCoeff m_line_model_;
};

#endif // PYSKELETON2GRAPH_INCLUDE_LINEAR_CLUSTER_H_