#ifndef PYSKELETON2GRAPH_INCLUDE_PIXEL_H_
#define PYSKELETON2GRAPH_INCLUDE_PIXEL_H_

#include <vector>
#include "typedef_pixel.h"
#include "enum.h"
#include "fast_arithmetic.h"

struct SkeletonPixel
{
    int32_t x_pos, y_pos;
    Hash hash;
    int32_t label;
    int8_t connectivity;
    PointType point_type;

    void setPosition(const int32_t &x_pos, const int32_t &y_pos)
    {
        this->x_pos = x_pos;
        this->y_pos = y_pos;
    }

    void getPosition(int32_t &x_pos, int32_t &y_pos) const
    {
        x_pos = this->x_pos;
        y_pos = this->y_pos;
    }

    void setHash(const Hash &hash)
    {
        this->hash = hash;
    }

    Hash getHash() const
    {
        return this->hash;
    }

    void setLabel(const int32_t &label)
    {
        this->label = label;
    }

    int32_t getLabel() const
    {
        return this->label;
    }

    void setConnectivity(const int8_t &connectivity)
    {
        this->connectivity = connectivity;
    }

    int8_t getConnectivity() const
    {
        return this->connectivity;
    }

    void setPointType()
    {
        if (this->connectivity < 0)
        {
            std::cout << "Connectivity point is not set" << std::endl;
            exit(EXIT_FAILURE);
        }
        switch (this->connectivity)
        {
        case 1:
            this->point_type = PointType::kEndPoint;
            break;
        case 2:
            this->point_type = PointType::kBridgePoint;
            break;
        default:
            if (this->connectivity > 2)
            {
                this->point_type = PointType::kJunctionPoint;
            }
            else
            {
                // this->connectivity == 0
                std::cout << "Invalid connectivity value is detected" << std::endl;
                exit(EXIT_FAILURE);
            }
            break;
        }
    }

    PointType getPointType() const
    {
        return this->point_type;
    }

    friend bool operator==(const SkeletonPixel &pixel1, const SkeletonPixel &pixel2)
    {
        return pixel1.hash == pixel2.hash;
    }

    friend bool operator!=(const SkeletonPixel &pixel1, const SkeletonPixel &pixel2)
    {
        return pixel1.hash == pixel2.hash;
    }

    /*
    return euclidean distance between two pixels
    */
    float distance(const SkeletonPixel &pixel_compare)
    {
        int32_t p1_x, p1_y, p2_x, p2_y;
        this->getPosition(p1_x, p1_y);
        pixel_compare.getPosition(p2_x, p2_y);
        return std::sqrt((p1_x - p2_x) * (p1_x - p2_x) + (p1_y - p2_y) * (p1_y - p2_y));
    }

    bool isNear(const SkeletonPixel &pixel_compare, const float &distance_threshold)
    {
        float dist = this->distance(pixel_compare);
        return dist <= distance_threshold;
    }

public:
    SkeletonPixel clone()
    {
        SkeletonPixel pixel;
        pixel.setPosition(x_pos, y_pos);
        pixel.setHash(hash);
        pixel.setLabel(hash);                // initial label <- initial node hash
        pixel.setConnectivity(connectivity); // and point type
        pixel.setPointType();
        return pixel;
    }
};

class EdgeSkeletonPixels
{
public:
    int32_t src;
    int32_t dst;
    EdgeSkeletonPixels(const SkeletonPixel &pixel_source, const SkeletonPixel &pixel_destination)
    {
        src = pixel_source.getHash();
        dst = pixel_destination.getHash();
        m_distance_between_two_pixels_ = calcPixelDistance(pixel_source, pixel_destination);
        m_angle_between_two_pixels_ = calcPixelDirection(pixel_source, pixel_destination);
    };

    EdgeSkeletonPixels(const SkeletonPixel *pixel_source, const SkeletonPixel *pixel_destination)
    {
        if (pixel_source == nullptr || pixel_destination == nullptr)
        {
            std::cout << "(pixel_source == nullptr || pixel_destination == nullptr)" << std::endl;
            exit(EXIT_FAILURE);
        }
        src = pixel_source->getHash();
        dst = pixel_destination->getHash();
        m_distance_between_two_pixels_ = calcPixelDistance(*pixel_source, *pixel_destination);
        m_angle_between_two_pixels_ = calcPixelDirection(*pixel_source, *pixel_destination);
    };

    void resetPixelPair(const SkeletonPixel &pixel_source, const SkeletonPixel &pixel_destination)
    {
        src = pixel_source.getHash();
        dst = pixel_destination.getHash();

        m_distance_between_two_pixels_ = calcPixelDistance(pixel_source, pixel_destination);
        if (pixel_source.getPointType() == PointType::kJunctionPoint || pixel_destination.getPointType() == PointType::kJunctionPoint)
        {
            m_angle_between_two_pixels_ = 10E8;
        }
    }

    float getEdgeLength()
    {
        return m_distance_between_two_pixels_;
    }

    float getEdgeAngle()
    {
        return m_angle_between_two_pixels_;
    }

private:
    float calcPixelDistance(const SkeletonPixel &pixel_a, const SkeletonPixel &pixel_b)
    {
        int32_t pixel_a_pos_x, pixel_a_pos_y;
        int32_t pixel_b_pos_x, pixel_b_pos_y;
        pixel_a.getPosition(pixel_a_pos_x, pixel_a_pos_y);
        pixel_b.getPosition(pixel_b_pos_x, pixel_b_pos_y);
        float x_diff = float(pixel_b_pos_x - pixel_a_pos_x);
        float y_diff = float(pixel_b_pos_y - pixel_a_pos_y);
        float dist_value = std::sqrt(x_diff * x_diff + y_diff * y_diff);
        return dist_value;
    }

    float calcPixelDirection(const SkeletonPixel &pixel_a, const SkeletonPixel &pixel_b)
    {
        int32_t pixel_a_pos_x, pixel_a_pos_y;
        int32_t pixel_b_pos_x, pixel_b_pos_y;
        pixel_a.getPosition(pixel_a_pos_x, pixel_a_pos_y);
        pixel_b.getPosition(pixel_b_pos_x, pixel_b_pos_y);
        float x_diff, y_diff;
        if (pixel_b_pos_y > pixel_a_pos_y)
        {
            x_diff = float(pixel_b_pos_x - pixel_a_pos_x);
            y_diff = float(pixel_b_pos_y - pixel_a_pos_y);
        }
        else
        {
            x_diff = float(pixel_a_pos_x - pixel_b_pos_x);
            y_diff = float(pixel_a_pos_y - pixel_b_pos_y);
        }
        float angle = fast_atan2f_2(y_diff, x_diff);
        //float angle = atan2f(y_diff, x_diff + 10E-8);
        return angle;
    }

    float m_distance_between_two_pixels_;
    float m_angle_between_two_pixels_;
};

#endif // PYSKELETON2GRAPH_INCLUDE_PIXEL_H_