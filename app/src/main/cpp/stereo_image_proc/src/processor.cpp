/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  [License text remains the same...]
*********************************************************************/
#include "processor.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <limits>
#include <iostream>
#include <cassert>
#include <cstring>

namespace stereo_image_proc {

// 图像编码常量定义
namespace image_encodings {
    const std::string MONO8 = "mono8";
    const std::string RGB8 = "rgb8";
    const std::string RGBA8 = "rgba8";
    const std::string BGR8 = "bgr8";
    const std::string BGRA8 = "bgra8";
    const std::string TYPE_32FC1 = "32FC1";
}

// CameraInfo实现
bool CameraInfo::loadFromYAML(const std::string& filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: Cannot open camera calibration file: " << filename << std::endl;
        return false;
    }
    
    try {
        // 读取基本参数
        fs["image_width"] >> image_width;
        fs["image_height"] >> image_height;
        fs["camera_name"] >> camera_name;
        
        // 读取相机矩阵
        cv::FileNode camera_matrix_node = fs["camera_matrix"];
        if (camera_matrix_node.type() == cv::FileNode::MAP) {
            std::vector<double> data;
            camera_matrix_node["data"] >> data;
            camera_matrix = cv::Mat(3, 3, CV_64F, data.data()).clone();
        }
        
        // 读取畸变系数
        cv::FileNode distortion_node = fs["distortion_coefficients"];
        if (distortion_node.type() == cv::FileNode::MAP) {
            std::vector<double> data;
            distortion_node["data"] >> data;
            distortion_coeffs = cv::Mat(1, data.size(), CV_64F, data.data()).clone();
        }
        
        // 读取校正矩阵
        cv::FileNode rectification_node = fs["rectification_matrix"];
        if (rectification_node.type() == cv::FileNode::MAP) {
            std::vector<double> data;
            rectification_node["data"] >> data;
            rectification_matrix = cv::Mat(3, 3, CV_64F, data.data()).clone();
        }
        
        // 读取投影矩阵
        cv::FileNode projection_node = fs["projection_matrix"];
        if (projection_node.type() == cv::FileNode::MAP) {
            std::vector<double> data;
            projection_node["data"] >> data;
            projection_matrix = cv::Mat(3, 4, CV_64F, data.data()).clone();
        }
        
    } catch (const cv::Exception& e) {
        std::cerr << "Error reading YAML file " << filename << ": " << e.what() << std::endl;
        return false;
    }
    
    fs.release();
    
    // 计算校正映射
    cv::Mat new_camera_matrix = projection_matrix(cv::Rect(0, 0, 3, 3));
    cv::initUndistortRectifyMap(camera_matrix, distortion_coeffs, rectification_matrix, 
                                new_camera_matrix, cv::Size(image_width, image_height), 
                                CV_16SC2, map1, map2);
    
    return true;
}

// StereoCameraModel实现
StereoCameraModel::StereoCameraModel() : fx_(0), fy_(0), cx_(0), cy_(0), baseline_(0) {
}

bool StereoCameraModel::loadFromYAMLFiles(const std::string& left_yaml, const std::string& right_yaml) {
    if (!left_cam_.loadFromYAML(left_yaml)) {
        std::cerr << "Failed to load left camera calibration from: " << left_yaml << std::endl;
        return false;
    }
    
    if (!right_cam_.loadFromYAML(right_yaml)) {
        std::cerr << "Failed to load right camera calibration from: " << right_yaml << std::endl;
        return false;
    }
    
    calculateStereoParameters();
    return true;
}

void StereoCameraModel::calculateStereoParameters() {
    // 从左相机的投影矩阵获取参数
    fx_ = left_cam_.projection_matrix.at<double>(0, 0);
    fy_ = left_cam_.projection_matrix.at<double>(1, 1);
    cx_ = left_cam_.projection_matrix.at<double>(0, 2);
    cy_ = left_cam_.projection_matrix.at<double>(1, 2);
    
    // 计算基线距离
    // 右相机投影矩阵的 P[0][3] = -fx * T，其中T是基线距离
    double tx = right_cam_.projection_matrix.at<double>(0, 3);
    baseline_ = -tx / fx_;
    
    // 构建重投影矩阵Q
    Q_ = cv::Mat::zeros(4, 4, CV_64F);
    Q_.at<double>(0, 0) = 1.0;
    Q_.at<double>(1, 1) = 1.0;
    Q_.at<double>(0, 3) = -cx_;
    Q_.at<double>(1, 3) = -cy_;
    Q_.at<double>(2, 3) = fx_;
    Q_.at<double>(3, 2) = -1.0 / baseline_;
    Q_.at<double>(3, 3) = (cx_ - right_cam_.projection_matrix.at<double>(0, 2)) / baseline_;
    
    std::cout << "Stereo parameters calculated:" << std::endl;
    std::cout << "  fx: " << fx_ << ", fy: " << fy_ << std::endl;
    std::cout << "  cx: " << cx_ << ", cy: " << cy_ << std::endl;
    std::cout << "  baseline: " << baseline_ << " meters" << std::endl;
}

void StereoCameraModel::rectifyImage(const cv::Mat& raw, cv::Mat& rectified, bool isLeft) const {
    if (isLeft) {
        cv::remap(raw, rectified, left_cam_.map1, left_cam_.map2, cv::INTER_LINEAR);
    } else {
        cv::remap(raw, rectified, right_cam_.map1, right_cam_.map2, cv::INTER_LINEAR);
    }
}

void StereoCameraModel::projectDisparityImageTo3d(const cv::Mat& disparity, cv::Mat& points3d, bool handleMissingValues) const {
    cv::reprojectImageTo3D(disparity, points3d, Q_, handleMissingValues);
    
    // 过滤无效点
    if (handleMissingValues) {
        cv::Mat_<cv::Vec3f> points = points3d;
        for (int y = 0; y < points.rows; ++y) {
            for (int x = 0; x < points.cols; ++x) {
                cv::Vec3f& pt = points(y, x);
                if (std::abs(pt[2]) > 1000.0f || pt[2] <= 0) {
                    pt[0] = pt[1] = pt[2] = MISSING_Z;
                }
            }
        }
    }
}

// 处理单目图像
bool processMonocular(const cv::Mat& raw_image, const StereoCameraModel& model, 
                     ImageSet& output, int flags, bool isLeft) {
    // 原始图像
    if (flags & StereoProcessor::LEFT_MONO || flags & StereoProcessor::RIGHT_MONO) {
        output.mono = raw_image.clone();
    }
    
    // 校正图像
    if (flags & StereoProcessor::LEFT_RECT || flags & StereoProcessor::RIGHT_RECT) {
        model.rectifyImage(raw_image, output.rect, isLeft);
    }
    
    // 彩色图像
    if (flags & StereoProcessor::LEFT_COLOR || flags & StereoProcessor::RIGHT_COLOR) {
        if (raw_image.channels() == 1) {
            cv::cvtColor(raw_image, output.color, cv::COLOR_GRAY2BGR);
            output.color_encoding = image_encodings::BGR8;
        } else {
            output.color = raw_image.clone();
            output.color_encoding = image_encodings::BGR8;
        }
    }
    
    // 校正后的彩色图像
    if (flags & StereoProcessor::LEFT_RECT_COLOR || flags & StereoProcessor::RIGHT_RECT_COLOR) {
        cv::Mat temp_color;
        if (raw_image.channels() == 1) {
            cv::cvtColor(raw_image, temp_color, cv::COLOR_GRAY2BGR);
        } else {
            temp_color = raw_image.clone();
        }
        model.rectifyImage(temp_color, output.rect_color, isLeft);
        output.color_encoding = image_encodings::BGR8;
    }
    
    return true;
}

bool StereoProcessor::process(const cv::Mat& left_raw,
                              const cv::Mat& right_raw,
                              const StereoCameraModel& model,
                              StereoImageSet& output, int flags) const
{
    // 对左右图像进行单目处理
    int left_flags = flags & LEFT_ALL;
    int right_flags = flags & RIGHT_ALL;
    
    if (flags & STEREO_ALL) {
        // 立体处理需要校正图像
        left_flags |= LEFT_RECT;
        right_flags |= RIGHT_RECT;
    }
    
    if (flags & (POINT_CLOUD | POINT_CLOUD2)) {
        flags |= DISPARITY;
        // 点云需要彩色通道
        left_flags |= LEFT_RECT_COLOR;
    }
    
    if (!processMonocular(left_raw, model, output.left, left_flags, true))
        return false;
    if (!processMonocular(right_raw, model, output.right, right_flags, false))
        return false;

    // 执行块匹配产生视差图
    if (flags & DISPARITY) {
        processDisparity(output.left.rect, output.right.rect, model, output.disparity);
    }

    // 将视差图投影到3D点云
    if (flags & POINT_CLOUD) {
        processPoints(output.disparity, output.left.rect_color, output.left.color_encoding, model, output.points);
    }

    // 将视差图投影到3D点云（格式2）
    if (flags & POINT_CLOUD2) {
        processPoints2(output.disparity, output.left.rect_color, output.left.color_encoding, model, output.points2);
    }

    return true;
}

void StereoProcessor::processDisparity(const cv::Mat& left_rect, const cv::Mat& right_rect,
                                       const StereoCameraModel& model,
                                       DisparityImage& disparity) const
{
    static const int DPP = 16;
    static const double inv_dpp = 1.0 / DPP;

    // 确保输入图像是灰度图
    cv::Mat left_gray, right_gray;
    if (left_rect.channels() == 3) {
        cv::cvtColor(left_rect, left_gray, cv::COLOR_BGR2GRAY);
    } else {
        left_gray = left_rect;
    }
    
    if (right_rect.channels() == 3) {
        cv::cvtColor(right_rect, right_gray, cv::COLOR_BGR2GRAY);
    } else {
        right_gray = right_rect;
    }

    // 块匹配器产生16位有符号视差图
    if (current_stereo_algorithm_ == BM) {
        block_matcher_->compute(left_gray, right_gray, disparity16_);
    } else {
        sg_block_matcher_->compute(left_gray, right_gray, disparity16_);
    }

    // 填充DisparityImage图像数据，转换为32位浮点
    ImageData& dimage = disparity.image;
    dimage.height = disparity16_.rows;
    dimage.width = disparity16_.cols;
    dimage.encoding = image_encodings::TYPE_32FC1;
    dimage.step = dimage.width * sizeof(float);
    dimage.data.resize(dimage.step * dimage.height);
    cv::Mat dmat(dimage.height, dimage.width, CV_32FC1, dimage.data.data(), dimage.step);
    
    // 从固定点转换为浮点视差
    disparity16_.convertTo(dmat, CV_32F, inv_dpp);
    assert(dmat.data == dimage.data.data());

    // 立体参数
    disparity.f = model.fx();
    disparity.T = model.baseline();

    // 视差搜索范围
    disparity.min_disparity = getMinDisparity();
    disparity.max_disparity = getMinDisparity() + getDisparityRange() - 1;
    disparity.delta_d = inv_dpp;
}

inline bool isValidPoint(const cv::Vec3f& pt) {
    return pt[2] != StereoCameraModel::MISSING_Z && !std::isinf(pt[2]) && !std::isnan(pt[2]) && pt[2] > 0;
}

void StereoProcessor::processPoints(const DisparityImage& disparity,
                                    const cv::Mat& color, const std::string& encoding,
                                    const StereoCameraModel& model,
                                    PointCloud& points) const
{
    // 计算密集点云
    const ImageData& dimage = disparity.image;
    cv::Mat dmat(dimage.height, dimage.width, CV_32FC1, (void*)dimage.data.data(), dimage.step);
    model.projectDisparityImageTo3d(dmat, dense_points_, true);

    // 填充稀疏点云消息
    points.points.clear();
    points.channels.resize(3);
    points.channel_names = {"rgb", "u", "v"};
    for (int i = 0; i < 3; ++i) {
        points.channels[i].clear();
    }
    
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
        for (int32_t v = 0; v < dense_points_.cols; ++v) {
            if (isValidPoint(dense_points_.at<cv::Vec3f>(u, v))) {
                Point3D pt;
                cv::Vec3f dense_pt = dense_points_.at<cv::Vec3f>(u, v);
                pt.x = dense_pt[0];
                pt.y = dense_pt[1];
                pt.z = dense_pt[2];
                points.points.push_back(pt);
                points.channels[1].push_back(u);
                points.channels[2].push_back(v);
            }
        }
    }

    // 填充颜色
    points.channels[0].reserve(points.points.size());
    if (encoding == image_encodings::BGR8) {
        for (int32_t u = 0; u < dense_points_.rows; ++u) {
            for (int32_t v = 0; v < dense_points_.cols; ++v) {
                if (isValidPoint(dense_points_.at<cv::Vec3f>(u, v))) {
                    const cv::Vec3b& bgr = color.at<cv::Vec3b>(u, v);
                    int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
                    points.channels[0].push_back(*(float*)(&rgb_packed));
                }
            }
        }
    }
    // 其他编码格式的处理...
}

void StereoProcessor::processPoints2(const DisparityImage& disparity,
                                     const cv::Mat& color, const std::string& encoding,
                                     const StereoCameraModel& model,
                                     PointCloud2& points) const
{
    // 计算密集点云
    const ImageData& dimage = disparity.image;
    cv::Mat dmat(dimage.height, dimage.width, CV_32FC1, (void*)dimage.data.data(), dimage.step);
    model.projectDisparityImageTo3d(dmat, dense_points_, true);

    // 填充点云消息
    points.height = dense_points_.rows;
    points.width = dense_points_.cols;
    points.fields.resize(4);
    points.fields[0].name = "x";
    points.fields[0].offset = 0;
    points.fields[0].count = 1;
    points.fields[0].datatype = PointCloud2::PointField::FLOAT32;
    points.fields[1].name = "y";
    points.fields[1].offset = 4;
    points.fields[1].count = 1;
    points.fields[1].datatype = PointCloud2::PointField::FLOAT32;
    points.fields[2].name = "z";
    points.fields[2].offset = 8;
    points.fields[2].count = 1;
    points.fields[2].datatype = PointCloud2::PointField::FLOAT32;
    points.fields[3].name = "rgb";
    points.fields[3].offset = 12;
    points.fields[3].count = 1;
    points.fields[3].datatype = PointCloud2::PointField::FLOAT32;
    points.point_step = 16;
    points.row_step = points.point_step * points.width;
    points.data.resize(points.row_step * points.height);
    points.is_dense = false;
 
    float bad_point = std::numeric_limits<float>::quiet_NaN();
    int i = 0;
    for (int32_t u = 0; u < dense_points_.rows; ++u) {
        for (int32_t v = 0; v < dense_points_.cols; ++v, ++i) {
            if (isValidPoint(dense_points_.at<cv::Vec3f>(u, v))) {
                cv::Vec3f dense_pt = dense_points_.at<cv::Vec3f>(u, v);
                memcpy(&points.data[i * points.point_step + 0], &dense_pt[0], sizeof(float));
                memcpy(&points.data[i * points.point_step + 4], &dense_pt[1], sizeof(float));
                memcpy(&points.data[i * points.point_step + 8], &dense_pt[2], sizeof(float));
                
                // 添加颜色信息
                if (encoding == image_encodings::BGR8) {
                    const cv::Vec3b& bgr = color.at<cv::Vec3b>(u, v);
                    int32_t rgb_packed = (bgr[2] << 16) | (bgr[1] << 8) | bgr[0];
                    memcpy(&points.data[i * points.point_step + 12], &rgb_packed, sizeof(int32_t));
                } else {
                    memcpy(&points.data[i * points.point_step + 12], &bad_point, sizeof(float));
                }
            }
            else {
                memcpy(&points.data[i * points.point_step + 0], &bad_point, sizeof(float));
                memcpy(&points.data[i * points.point_step + 4], &bad_point, sizeof(float));
                memcpy(&points.data[i * points.point_step + 8], &bad_point, sizeof(float));
                memcpy(&points.data[i * points.point_step + 12], &bad_point, sizeof(float));
            }
        }
    }
}

} // namespace stereo_image_proc