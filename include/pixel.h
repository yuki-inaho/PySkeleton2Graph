#ifndef PYSKELETON2GRAPH_INCLUDE_PIXEL_H_
#define PYSKELETON2GRAPH_INCLUDE_PIXEL_H_

#include <vector>
#include "typedef.h"
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
        //std::cout << src << " " << dst << std::endl;
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

    void resetPixelPair(const SkeletonPixel &pixel_source, const SkeletonPixel &pixel_destination){
        src = pixel_source.getHash();
        dst = pixel_destination.getHash();
        m_distance_between_two_pixels_ = calcPixelDistance(pixel_source, pixel_destination);
        m_angle_between_two_pixels_ = calcPixelDirection(pixel_source, pixel_destination);
    }

    float get_edge_length(){
        return m_distance_between_two_pixels_;
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
        float x_diff = float(pixel_b_pos_x - pixel_a_pos_x);
        float y_diff = float(pixel_b_pos_y - pixel_a_pos_y);
        float angle = fast_atan2f_2(y_diff, x_diff);
        return angle;
    }

    float m_distance_between_two_pixels_;
    float m_angle_between_two_pixels_;
};

#endif // PYSKELETON2GRAPH_INCLUDE_PIXEL_H_