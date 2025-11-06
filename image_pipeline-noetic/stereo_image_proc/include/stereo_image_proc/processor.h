/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  [License text remains the same...]
*********************************************************************/
#ifndef STEREO_IMAGE_PROC_PROCESSOR_H
#define STEREO_IMAGE_PROC_PROCESSOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
#include <memory>
#include <limits>

namespace stereo_image_proc {

// 相机标定参数结构
struct CameraInfo {
    int image_width;
    int image_height;
    std::string camera_name;
    cv::Mat camera_matrix;      // 3x3 内参矩阵
    cv::Mat distortion_coeffs;  // 1x5 畸变系数
    cv::Mat rectification_matrix; // 3x3 校正矩阵
    cv::Mat projection_matrix;  // 3x4 投影矩阵
    
    // 用于图像校正的映射
    cv::Mat map1, map2;
    
    bool loadFromYAML(const std::string& filename);
};

// 替代ROS消息的简单数据结构
struct ImageData {
    int height;
    int width;
    std::string encoding;
    size_t step;
    std::vector<uint8_t> data;
};

struct DisparityImage {
    ImageData image;
    float f;           // 焦距
    float T;           // 基线
    float min_disparity;
    float max_disparity;
    float delta_d;
};

struct Point3D {
    float x, y, z;
};

struct PointCloud {
    std::vector<Point3D> points;
    std::vector<std::vector<float>> channels; // [rgb, u, v]
    std::vector<std::string> channel_names;
};

struct PointCloud2 {
    int height;
    int width;
    struct PointField {
        std::string name;
        int offset;
        int count;
        int datatype;
        enum { FLOAT32 = 7 };
    };
    std::vector<PointField> fields;
    int point_step;
    int row_step;
    std::vector<uint8_t> data;
    bool is_dense;
};

struct ImageSet {
    cv::Mat mono;
    cv::Mat rect;
    cv::Mat color;
    cv::Mat rect_color;
    std::string color_encoding;
};

struct StereoImageSet {
    ImageSet left;
    ImageSet right;
    DisparityImage disparity;
    PointCloud points;
    PointCloud2 points2;
};

// 立体相机模型
class StereoCameraModel {
public:
    StereoCameraModel();
    
    // 从YAML文件加载标定参数
    bool loadFromYAMLFiles(const std::string& left_yaml, const std::string& right_yaml);
    
    // 获取参数
    float fx() const { return fx_; }
    float fy() const { return fy_; }
    float cx() const { return cx_; }
    float cy() const { return cy_; }
    float baseline() const { return baseline_; }
    
    // 获取左右相机参数
    const CameraInfo& left() const { return left_cam_; }
    const CameraInfo& right() const { return right_cam_; }
    
    // 3D投影
    void projectDisparityImageTo3d(const cv::Mat& disparity, cv::Mat& points3d, bool handleMissingValues = true) const;
    
    // 图像校正
    void rectifyImage(const cv::Mat& raw, cv::Mat& rectified, bool isLeft) const;
    
    static constexpr float MISSING_Z = std::numeric_limits<float>::quiet_NaN();

private:
    void calculateStereoParameters();
    
    CameraInfo left_cam_;
    CameraInfo right_cam_;
    
    // 立体参数
    float fx_, fy_, cx_, cy_;
    float baseline_;
    cv::Mat Q_; // 重投影矩阵
};

class StereoProcessor {
public:
    StereoProcessor() : current_stereo_algorithm_(BM) {
        block_matcher_ = cv::StereoBM::create();
        sg_block_matcher_ = cv::StereoSGBM::create(1, 1, 10);
    }

    enum StereoType {
        BM, SGBM
    };

    enum {
        LEFT_MONO        = 1 << 0,
        LEFT_RECT        = 1 << 1,
        LEFT_COLOR       = 1 << 2,
        LEFT_RECT_COLOR  = 1 << 3,
        RIGHT_MONO       = 1 << 4,
        RIGHT_RECT       = 1 << 5,
        RIGHT_COLOR      = 1 << 6,
        RIGHT_RECT_COLOR = 1 << 7,
        DISPARITY        = 1 << 8,
        POINT_CLOUD      = 1 << 9,
        POINT_CLOUD2     = 1 << 10,

        LEFT_ALL = LEFT_MONO | LEFT_RECT | LEFT_COLOR | LEFT_RECT_COLOR,
        RIGHT_ALL = RIGHT_MONO | RIGHT_RECT | RIGHT_COLOR | RIGHT_RECT_COLOR,
        STEREO_ALL = DISPARITY | POINT_CLOUD | POINT_CLOUD2,
        ALL = LEFT_ALL | RIGHT_ALL | STEREO_ALL
    };

    // 基本接口
    StereoType getStereoType() const { return current_stereo_algorithm_; }
    void setStereoType(StereoType type) { current_stereo_algorithm_ = type; }

    // 参数设置接口
    int getPreFilterSize() const;
    void setPreFilterSize(int size);
    int getPreFilterCap() const;
    void setPreFilterCap(int cap);
    int getCorrelationWindowSize() const;
    void setCorrelationWindowSize(int size);
    int getMinDisparity() const;
    void setMinDisparity(int min_d);
    int getDisparityRange() const;
    void setDisparityRange(int range);
    int getTextureThreshold() const;
    void setTextureThreshold(int threshold);
    float getUniquenessRatio() const;
    void setUniquenessRatio(float ratio);
    int getSpeckleSize() const;
    void setSpeckleSize(int size);
    int getSpeckleRange() const;
    void setSpeckleRange(int range);

    // SGBM专用参数
    int getSgbmMode() const;
    void setSgbmMode(int mode);
    int getP1() const;
    void setP1(int P1);
    int getP2() const;
    void setP2(int P2);
    int getDisp12MaxDiff() const;
    void setDisp12MaxDiff(int disp12MaxDiff);

    // 主要处理函数
    bool process(const cv::Mat& left_raw,
                 const cv::Mat& right_raw,
                 const StereoCameraModel& model,
                 StereoImageSet& output, int flags) const;

    void processDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect,
                          const StereoCameraModel& model,
                          DisparityImage& disparity) const;

    void processPoints(const DisparityImage& disparity,
                       const cv::Mat& color, const std::string& encoding,
                       const StereoCameraModel& model,
                       PointCloud& points) const;

    void processPoints2(const DisparityImage& disparity,
                        const cv::Mat& color, const std::string& encoding,
                        const StereoCameraModel& model,
                        PointCloud2& points) const;

private:
    mutable cv::Mat disparity16_;
    mutable cv::Ptr<cv::StereoBM> block_matcher_;
    mutable cv::Ptr<cv::StereoSGBM> sg_block_matcher_;
    StereoType current_stereo_algorithm_;
    mutable cv::Mat dense_points_;
};

// 内联函数实现
inline int StereoProcessor::getPreFilterCap() const {
    if (current_stereo_algorithm_ == BM)
        return block_matcher_->getPreFilterCap();
    return sg_block_matcher_->getPreFilterCap();
}

inline void StereoProcessor::setPreFilterCap(int cap) {
    block_matcher_->setPreFilterCap(cap);
    sg_block_matcher_->setPreFilterCap(cap);
}

inline int StereoProcessor::getCorrelationWindowSize() const {
    if (current_stereo_algorithm_ == BM)
        return block_matcher_->getBlockSize();
    return sg_block_matcher_->getBlockSize();
}

inline void StereoProcessor::setCorrelationWindowSize(int size) {
    block_matcher_->setBlockSize(size);
    sg_block_matcher_->setBlockSize(size);
}

inline int StereoProcessor::getMinDisparity() const {
    if (current_stereo_algorithm_ == BM)
        return block_matcher_->getMinDisparity();
    return sg_block_matcher_->getMinDisparity();
}

inline void StereoProcessor::setMinDisparity(int min_d) {
    block_matcher_->setMinDisparity(min_d);
    sg_block_matcher_->setMinDisparity(min_d);
}

inline int StereoProcessor::getDisparityRange() const {
    if (current_stereo_algorithm_ == BM)
        return block_matcher_->getNumDisparities();
    return sg_block_matcher_->getNumDisparities();
}

inline void StereoProcessor::setDisparityRange(int range) {
    block_matcher_->setNumDisparities(range);
    sg_block_matcher_->setNumDisparities(range);
}

inline float StereoProcessor::getUniquenessRatio() const {
    if (current_stereo_algorithm_ == BM)
        return block_matcher_->getUniquenessRatio();
    return sg_block_matcher_->getUniquenessRatio();
}

inline void StereoProcessor::setUniquenessRatio(float ratio) {
    block_matcher_->setUniquenessRatio(ratio);
    sg_block_matcher_->setUniquenessRatio(ratio);
}

inline int StereoProcessor::getSpeckleSize() const {
    if (current_stereo_algorithm_ == BM)
        return block_matcher_->getSpeckleWindowSize();
    return sg_block_matcher_->getSpeckleWindowSize();
}

inline void StereoProcessor::setSpeckleSize(int size) {
    block_matcher_->setSpeckleWindowSize(size);
    sg_block_matcher_->setSpeckleWindowSize(size);
}

inline int StereoProcessor::getSpeckleRange() const {
    if (current_stereo_algorithm_ == BM)
        return block_matcher_->getSpeckleRange();
    return sg_block_matcher_->getSpeckleRange();
}

inline void StereoProcessor::setSpeckleRange(int range) {
    block_matcher_->setSpeckleRange(range);
    sg_block_matcher_->setSpeckleRange(range);
}

inline int StereoProcessor::getPreFilterSize() const {
    return block_matcher_->getPreFilterSize();
}

inline void StereoProcessor::setPreFilterSize(int size) {
    block_matcher_->setPreFilterSize(size);
}

inline int StereoProcessor::getTextureThreshold() const {
    return block_matcher_->getTextureThreshold();
}

inline void StereoProcessor::setTextureThreshold(int threshold) {
    block_matcher_->setTextureThreshold(threshold);
}

inline int StereoProcessor::getSgbmMode() const {
    return sg_block_matcher_->getMode();
}

inline void StereoProcessor::setSgbmMode(int mode) {
    sg_block_matcher_->setMode(mode);
}

inline int StereoProcessor::getP1() const {
    return sg_block_matcher_->getP1();
}

inline void StereoProcessor::setP1(int P1) {
    sg_block_matcher_->setP1(P1);
}

inline int StereoProcessor::getP2() const {
    return sg_block_matcher_->getP2();
}

inline void StereoProcessor::setP2(int P2) {
    sg_block_matcher_->setP2(P2);
}

inline int StereoProcessor::getDisp12MaxDiff() const {
    return sg_block_matcher_->getDisp12MaxDiff();
}

inline void StereoProcessor::setDisp12MaxDiff(int disp12MaxDiff) {
    sg_block_matcher_->setDisp12MaxDiff(disp12MaxDiff);
}

} // namespace stereo_image_proc

#endif