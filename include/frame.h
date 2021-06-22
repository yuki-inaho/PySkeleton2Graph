#ifndef PYSKELETON2GRAPH_INCLUDE_FRAME_H_
#define PYSKELETON2GRAPH_INCLUDE_FRAME_H_

#include <iostream>
#include <opencv2/core/core.hpp>

class SkeletonFrame
{
public:
    SkeletonFrame(const cv::Mat &Skeleton_image)
    {
        SetImage(Skeleton_image);
    }

    std::vector<float> data;
    int32_t image_width, image_height;
    unsigned char max_value, min_value;

private:
    void SetImage(const cv::Mat &Skeleton_image)
    {
        TypeValidationSkeletonImage(Skeleton_image);
        Skeleton_image_ = Skeleton_image;
        image_width = Skeleton_image.cols;
        image_height = Skeleton_image.rows;

        double max_value_d, min_value_d;
        cv::minMaxLoc(Skeleton_image, &min_value_d, &max_value_d);
        min_value = static_cast<unsigned char>(min_value_d);
        max_value = static_cast<unsigned char>(max_value_d);

        std::vector<float> data_(image_width * image_height, 0);
        for (int32_t v = 0; v < image_height; v++)
        {
            for (int32_t u = 0; u < image_width; u++)
            {
                int32_t k = v * image_width + u;
                data_[k] = static_cast<float>(Skeleton_image.at<unsigned char>(v, u));
            }
        }
        data = data_;
    }

    void TypeValidationSkeletonImage(const cv::Mat Skeleton_image)
    {
        if (Skeleton_image.type() != CV_8UC1)
        {
            std::cerr << "Skeleton_image.channels() != CV_8UC1" << std::endl;
            std::exit(EXIT_FAILURE);
        }
    }

    cv::Mat Skeleton_image_;
};

#endif // PYSKELETON2GRAPH_INCLUDE_FRAME_H_