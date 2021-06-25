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

typedef Node<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraphNode;
typedef Edge<SkeletonPixel, EdgeSkeletonPixels> SkeletonGraphEdge;
typedef GraphHelper<SkeletonPixel, EdgeSkeletonPixels> *SkeletonGraphHelperPtr;
typedef Eigen::Matrix<float, 3, Eigen::Dynamic> Matrix2DPoints;

// TODO: rename "m_node_list_"
struct LineCoeff
{
    float n_x;
    float n_y;
    float constant;

    void set(float p1_x, float p1_y, float p2_x, float p2_y)
    {
        float l_x = p2_x - p1_x;
        float l_y = p2_y - p1_y;

        float l_sqrt = std::sqrt(l_x * l_x + l_y * l_y);
        l_x /= l_sqrt;
        l_y /= l_sqrt;

        n_x = -l_y;
        n_y = l_x;

        /// align vector direction
        if (n_y > 0)
        {
            n_x *= -1;
            n_y *= -1;
        }
        constant = -(n_x * p1_x + n_y * p1_y);
    }

    void set(float n_x_, float n_y_, float constant_)
    {
        float n_norm = std::sqrt(n_x_ * n_x_ + n_y_ * n_y_);
        n_x = n_x_ / n_norm;
        n_y = n_y_ / n_norm;

        /// align vector direction
        if (n_y > 0)
        {
            n_x *= -1;
            n_y *= -1;
        }
        constant = constant_ / n_norm;
    }

    float get_fit_error(float p_x, float p_y)
    {
        return n_x * p_x + n_y * p_y + constant;
    }
};

class LinearCluster
{
public:
    LinearCluster(int32_t cluster_label, SkeletonGraphHelperPtr graph_helper, float cluster_proximity_threshold) : m_parent_(this), m_cluster_label_(cluster_label), m_graph_helper_ptr_(graph_helper), m_cluster_proximity_threshold_(cluster_proximity_threshold), m_has_junction_point_(false), m_has_end_point_(false){};
    ~LinearCluster(){};

    int32_t label() const
    {
        return m_cluster_label_;
    }

    int32_t size() const
    {
        return m_node_list_.size();
    }

    std::shared_ptr<SkeletonGraphNode> accessNodeByIndex(int32_t index)
    {
        if ((index < 0) || (index >= size()))
        {
            std::cerr << "Invalid cluster point access detected" << std::endl;
            exit(EXIT_FAILURE);
        }
        return m_node_list_[index];
    }

    void addNodePtr(SkeletonGraphNode *node_ptr)
    {
        std::shared_ptr<SkeletonGraphNode> node_shared_ptr(node_ptr);
        if(node_ptr->data.getPointType() == PointType::kEndPoint){
            m_has_end_point_ = true;
        }else if(node_ptr->data.getPointType() == PointType::kJunctionPoint){
            m_has_junction_point_ = true;
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
            fitLineBySVD(m_line_model_);
        }
    }

    float getMeanSquaredError()
    {
        float sum_error = 0;
        int32_t n_points = m_node_list_.size();
        for (int32_t i = 0; i < n_points; i++)
        {
            int32_t px, py;
            m_node_list_[i]->data.getPosition(px, py);
            sum_error += m_line_model_.get_fit_error(px, py);
        }
        return sum_error / float(n_points);
    }

    void normal(float &n_x, float &n_y) const
    {
        n_x = m_line_model_.n_x;
        n_y = m_line_model_.n_y;
    }

    float getDiffDegree(const LinearCluster *line_model_compare)
    {
        float n_this_x = m_line_model_.n_x;
        float n_this_y = m_line_model_.n_y;
        float n_comp_x, n_comp_y;
        line_model_compare->normal(n_comp_x, n_comp_y);
        float inner_product_n = n_this_x * n_comp_x + n_this_y * n_comp_y;
        return std::acos(inner_product_n) / M_PI * 180.0;
    }

    float getProjected1DPoint(const int32_t &p_x, const int32_t &p_y, int32_t &p_x_projected, int32_t &p_y_projected)
    {
        float error = calcPointProjectionError(p_x, p_y);
        p_x_projected = static_cast<int32_t>(p_x - error * m_line_model_.n_x);
        p_y_projected = static_cast<int32_t>(p_y - error * m_line_model_.n_y);
        std::cout << calcPointProjectionError(p_x_projected, p_y_projected) << std::endl;
    }

    /*
        not absoluted
    */
    float calcPointProjectionError(const float &p_x, const float &p_y)
    {
        float n_x = m_line_model_.n_x;
        float n_y = m_line_model_.n_y;
        float c = m_line_model_.constant;
        return n_x * p_x + n_y * p_y + c;
    }

    float calcTwoPointDistanceOnLine(const float &p0_x, const float &p0_y, const float &p1_x, const float &p1_y)
    {
        float n_x = m_line_model_.n_x;
        float n_y = m_line_model_.n_y;
        float diff_x = p1_x - p0_x;
        float diff_y = p1_y - p0_y;
        return n_x * diff_x + n_y * diff_y;
    }

    // FORME: use reference?
    bool isClusterNeighbor(LinearCluster* cluster_compare)
    {
        for (std::shared_ptr<SkeletonGraphNode> node_ptr_this : m_node_list_)
        {
            int32_t p_x_this, p_y_this;
            node_ptr_this->data.getPosition(p_x_this, p_y_this);
            if (cluster_compare->isPointNeighbor(p_x_this, p_y_this))
            {
                return true;
            }
        }
        return false;
    }

    bool isPointNeighbor(const int32_t& p_x, const int32_t& p_y)
    {
        for (std::shared_ptr<SkeletonGraphNode> node_ptr_this : m_node_list_)
        {
            int32_t p_x_this, p_y_this;
            node_ptr_this->data.getPosition(p_x_this, p_y_this);
            if (arePointsMutual(p_x_this, p_y_this, p_x, p_y))
            {
                return true;
            }
        }
        return false;
    }

private:
    bool arePointsMutual(float p1_x, float p1_y, float p2_x, float p2_y)
    {
        float diff_x = p2_x - p1_x;
        float diff_y = p2_y - p1_y;
        return std::sqrt(diff_x * diff_x + diff_y * diff_y) <= m_cluster_proximity_threshold_;
    }

    void fitLineBySVD(LineCoeff &line_coeff)
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
        line_coeff.set(u(0, 2), u(1, 2), u(2, 2)); // get eigen vector corresponded with the smallast eigen value
    }

    bool m_has_end_point_;
    bool m_has_junction_point_;
    int32_t m_cluster_label_;
    float m_cluster_proximity_threshold_;
    std::vector<std::shared_ptr<SkeletonGraphNode>> m_node_list_;
    LinearCluster *m_parent_;
    SkeletonGraphHelperPtr m_graph_helper_ptr_;
    LineCoeff m_line_model_;
};

#endif // PYSKELETON2GRAPH_INCLUDE_LINEAR_CLUSTER_H_