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

// TODO: rename "m_node_list_"
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
    LinearCluster(int32_t cluster_label, const SkeletonGraphHelperPtr graph_helper) : m_parent_(this), m_cluster_label_(cluster_label), m_graph_helper_ptr_(graph_helper), m_cluster_type_(LinearClusterType::kBridgePointCluster){};
    ~LinearCluster(){};

    int32_t label() const
    {
        return m_cluster_label_;
    }

    void relabel(const int32_t &new_label)
    {
        m_cluster_label_ = new_label;
        for (std::shared_ptr<SkeletonGraphNode> node : m_node_list_)
        {
            node->data.setLabel(new_label);
        }
    }

    int32_t size() const
    {
        return m_node_list_.size();
    }

    LinearClusterType type() const
    {
        return m_cluster_type_;
    }

    /*
    Return fitted line model parameters
    */
    std::vector<float> line()
    {
        std::vector<float> parameters{m_line_model_.n_x(), m_line_model_.n_y(), m_line_model_.n_constant()};
        return parameters;
    }

    /*
    Return skeleton point positions(px, py)
    */
    std::vector<std::vector<int32_t>> points()
    {
        std::vector<std::vector<int32_t>> points;
        for (std::shared_ptr<SkeletonGraphNode> node_ptr : m_node_list_)
        {
            int32_t px, py;
            node_ptr->data.getPosition(px, py);
            std::vector<int32_t> point{px, py};
            points.push_back(point);
        }
        return points;
    }

    std::vector<int32_t> getEndPointIndices() const
    {
        std::vector<int32_t> indices;
        int32_t index = 0;
        for (std::shared_ptr<SkeletonGraphNode> node_ptr : m_node_list_)
        {
            if (isEndPoint(node_ptr))
                indices.push_back(index);
            index++;
        }
        return indices;
    }

    /*
    mainly for debug
    */
    std::vector<std::vector<int32_t>> edges() const
    {
        std::unordered_map<Hash, int32_t> map_hash2index;
        for (int32_t index = 0; index < m_node_list_.size(); index++)
        {
            map_hash2index.insert({m_node_list_[index]->data.getHash(), index});
        }

        std::vector<std::vector<int32_t>> edges;
        for (std::shared_ptr<SkeletonGraphNode> node_ptr : m_node_list_)
        {
            for (auto edge_ptr = node_ptr->firstOut; edge_ptr; edge_ptr = edge_ptr->nextInFrom)
            {
                if (map_hash2index.count(edge_ptr->data.dst) == 0)
                    continue;
                std::vector<int32_t> edge{map_hash2index.at(edge_ptr->data.src), map_hash2index.at(edge_ptr->data.dst)};
                edges.push_back(edge);
            }
        }
        return edges;
    }

    void addNodePtr(SkeletonGraphNode *node_ptr)
    {
        std::shared_ptr<SkeletonGraphNode> node_shared_ptr(node_ptr);
        /// FORME: confirm in the case of kEndPointCluster + kJunctionPointCluster
        if (node_ptr->data.getPointType() == PointType::kEndPoint)
        {
            m_cluster_type_ = LinearClusterType::kEndPointCluster;
        }
        else if (node_ptr->data.getPointType() == PointType::kJunctionPoint)
        {
            m_cluster_type_ = LinearClusterType::kJunctionPointCluster;
        }
        m_node_list_.push_back(node_shared_ptr);
    }

    void fitLine()
    {
        if (size() == 1)
        {
            std::cerr << "Number of cluster point is too small to fit 2D-line" << std::endl;
            int32_t px, py;
            m_node_list_[0]->data.getPosition(px, py);
            m_line_model_.set(1.0, 0, px, py);
        }
        else if (size() == 2)
        {
            int32_t p0x, p0y;
            m_node_list_[0]->data.getPosition(p0x, p0y);
            int32_t p1x, p1y;
            m_node_list_[1]->data.getPosition(p1x, p1y);
            m_line_model_.set(p0x, p0y, p1x, p1y);
        }
        else
        {
            float n_x, n_y, n_constant;
            fitLineBySVD(n_x, n_y, n_constant);
            m_line_model_.set(n_x, n_y, n_constant);
        }
    }

    float getDiffDegree(LinearCluster *line_model_compare)
    {
        float n_this_x = m_line_model_.n_x();
        float n_this_y = m_line_model_.n_y();
        float n_comp_x, n_comp_y;
        line_model_compare->normal(n_comp_x, n_comp_y);
        float inner_product_n = n_this_x * n_comp_x + n_this_y * n_comp_y;

        return std::acos(inner_product_n) / M_PI * 180.0;
    }

    float getFittingError(const float &p_x, const float &p_y)
    {
        return m_line_model_.get_fit_error(p_x, p_y);
    }

    float getMeanSquaredError()
    {
        float sum_error = 0;

        // TODO: Use reduce
        int32_t n_points = m_node_list_.size();
        for (int32_t i = 0; i < n_points; i++)
        {
            int32_t px, py;
            m_node_list_[i]->data.getPosition(px, py);
            sum_error += getFittingError(px, py);
        }
        return sum_error / n_points;
    }

private:
    void normal(float &n_x, float &n_y)
    {
        n_x = m_line_model_.n_x();
        n_y = m_line_model_.n_y();
    }

    void fitLineBySVD(float &n_x_output, float &n_y_output, float &constant_output)
    {
        Matrix2DPoints mat_points;
        mat_points.resize(3, m_node_list_.size());
        int32_t n_points = m_node_list_.size();
        for (int32_t i = 0; i < n_points; i++)
        {
            int32_t px, py;
            m_node_list_[i]->data.getPosition(px, py);
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

    bool isEndPoint(const std::shared_ptr<SkeletonGraphNode> node_ptr) const
    {
        if (node_ptr->data.getPointType() == PointType::kEndPoint)
            return true;
        for (auto edge_ptr = node_ptr->firstOut; edge_ptr; edge_ptr = edge_ptr->nextInFrom)
        {
            int32_t label_dst = m_graph_helper_ptr_->getNodePtr(edge_ptr->data.dst)->data.getLabel();
            if (m_cluster_label_ != label_dst)
                return true;
        }
        return false;
    }

    LinearClusterType m_cluster_type_;
    int32_t m_cluster_label_;
    float m_cluster_proximity_threshold_;
    std::vector<std::shared_ptr<SkeletonGraphNode>> m_node_list_;
    LinearCluster *m_parent_;
    SkeletonGraphHelperPtr m_graph_helper_ptr_;
    LineCoeff m_line_model_;
};

#endif // PYSKELETON2GRAPH_INCLUDE_LINEAR_CLUSTER_H_