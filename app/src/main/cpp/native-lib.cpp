//native-lib.cpp
#include <jni.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iterator>
#include <ctime>
#include <vector>
#include <set>
#include <thread>
#include <mutex>
#include <random>
#include <cmath>
#include <sstream>
#include <iomanip>
#include "PointCloudPublisher.h"
// 必须先包含Eigen头文件
#include <Eigen/Dense>

// 然后包含OpenCV头文件
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/eigen.hpp>

#include "System.h"
#include <opencv2/imgproc.hpp>
#include "astar/Astar.h"
// Android logging
#ifdef LOG_TAG
#undef LOG_TAG
#endif

#include <android/log.h>
#define LOG_TAG "ORB_SLAM3"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)

// =============================================================================
// 全局变量
// =============================================================================

ORB_SLAM3::System* SLAM = nullptr;
std::chrono::steady_clock::time_point t0;
double tframe = 0;
static long frameCount = 0;

// =============================================================================
// 辅助函数
// =============================================================================

// 检查文件是否存在
bool fileExists(const std::string& path) {
    std::ifstream file(path);
    return file.good();
}

// 检查可用内存 (MB)
int getAvailableMemory() {
    FILE* fp = fopen("/proc/meminfo", "r");
    if (fp == nullptr)
        return 1000; // 假设有足够内存

    char line[128];
    int availableMemoryMB = 0;

    while (fgets(line, sizeof(line), fp)) {
        if (strncmp(line, "MemAvailable:", 13) == 0) {
            long mem_kb;
            sscanf(line + 13, "%ld", &mem_kb);
            availableMemoryMB = mem_kb / 1024;
            break;
        }
    }
    fclose(fp);

    return availableMemoryMB;
}

// 从文本加载词汇表
bool load_as_text(ORB_SLAM3::ORBVocabulary* voc, const std::string infile) {
    if (!fileExists(infile)) {
        LOGE("Vocabulary file not found: %s", infile.c_str());
        return false;
    }

    clock_t tStart = clock();
    bool res = voc->loadFromTextFile(infile);
    LOGI("Loading from text: %.2fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    return res;
}

// 保存为二进制格式
void save_as_binary(ORB_SLAM3::ORBVocabulary* voc, const std::string outfile) {
    clock_t tStart = clock();
    voc->saveToBinaryFile(outfile);
    LOGI("Saving as binary: %.2fs", (double)(clock() - tStart)/CLOCKS_PER_SEC);
}

// 将文本词汇表转换为二进制格式
bool convertVocabulary(const std::string& textPath, const std::string& binaryPath) {
    LOGI("Starting vocabulary conversion from '%s' to '%s'", textPath.c_str(), binaryPath.c_str());

    if (!fileExists(textPath)) {
        LOGE("Input vocabulary file not found: %s", textPath.c_str());
        return false;
    }

    if (fileExists(binaryPath)) {
        LOGI("Binary vocabulary already exists: %s", binaryPath.c_str());
        return true;
    }

    ORB_SLAM3::ORBVocabulary voc;

    LOGI("Loading vocabulary from text file");
    clock_t tStart = clock();
    bool loaded;

    try {
        loaded = voc.loadFromTextFile(textPath);
    } catch (const std::exception& e) {
        LOGE("Exception loading vocabulary: %s", e.what());
        return false;
    }

    double loadTime = (double)(clock() - tStart)/CLOCKS_PER_SEC;
    LOGI("Loading time: %.2f seconds", loadTime);

    if (!loaded) {
        LOGE("Failed to load vocabulary from text file");
        return false;
    }

    LOGI("Vocabulary loaded successfully, saving to binary file");
    tStart = clock();

    try {
        voc.saveToBinaryFile(binaryPath);
    } catch (const std::exception& e) {
        LOGE("Exception saving vocabulary: %s", e.what());
        return false;
    }

    double saveTime = (double)(clock() - tStart)/CLOCKS_PER_SEC;
    LOGI("Saving time: %.2f seconds", saveTime);

    if (!fileExists(binaryPath)) {
        LOGE("Failed to create binary vocabulary file");
        return false;
    }

    LOGI("Vocabulary conversion completed successfully");
    return true;
}

// =============================================================================
// JNI 函数实现
// =============================================================================

// 测试 OpenMP 功能
extern "C" JNIEXPORT jstring JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_testOpenMP(
        JNIEnv* env,
        jobject /* this */) {
    LOGI("Testing OpenMP support...");

    int sum = 0;
    for (int i = 0; i < 1000; i++) {
        sum += i;
    }

    LOGI("Test completed without OpenMP. Sum: %d", sum);

    std::string result = "Test completed without OpenMP. Sum: " + std::to_string(sum);
    return env->NewStringUTF(result.c_str());
}

// 测试SLAM内存使用
extern "C" JNIEXPORT jstring JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_testSLAMMemory(
        JNIEnv* env,
        jobject /* this */) {

    int availMem = getAvailableMemory();

    std::string result = "Memory Test Results:\n";
    result += "Available memory: " + std::to_string(availMem) + " MB\n";

    int mapPoints = 0;

    if (SLAM != nullptr) {
        result += "SLAM status: Initialized\n";

        int trackingState = SLAM->GetTrackingState();
        result += "Tracking state: " + std::to_string(trackingState) + "\n";

        try {
            if (SLAM->mpTracker && SLAM->mpTracker->mpAtlas) {
                const std::vector<ORB_SLAM3::MapPoint*> vpMPs = SLAM->mpTracker->mpAtlas->GetAllMapPoints();
                mapPoints = vpMPs.size();
            }
            result += "Map points: " + std::to_string(mapPoints) + "\n";
        } catch (...) {
            result += "Error getting map points\n";
        }
    } else {
        result += "SLAM status: Not initialized\n";
    }


    result += "Grid map builder: Managed by Java layer\n";

    int estimatedMemory = 0;
    if (SLAM != nullptr) {
        estimatedMemory = 100 + (mapPoints / 10);
        result += "Estimated SLAM memory: ~" + std::to_string(estimatedMemory) + " MB\n";
    }

    if (availMem < 200) {
        result += "Warning: Low memory available!\n";
    } else if (availMem > 500) {
        result += "Memory status: Good\n";
    } else {
        result += "Memory status: Adequate\n";
    }

    return env->NewStringUTF(result.c_str());
}

// 转换词汇表
extern "C" JNIEXPORT jboolean JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_convertVocabulary(
        JNIEnv* env,
        jobject /* this */,
        jstring inputPath,
        jstring outputPath) {

    const char* inputPathCStr = env->GetStringUTFChars(inputPath, nullptr);
    const char* outputPathCStr = env->GetStringUTFChars(outputPath, nullptr);

    if (!inputPathCStr || !outputPathCStr) {
        LOGE("Failed to get path strings");
        if (inputPathCStr) env->ReleaseStringUTFChars(inputPath, inputPathCStr);
        if (outputPathCStr) env->ReleaseStringUTFChars(outputPath, outputPathCStr);
        return JNI_FALSE;
    }

    std::string inputPathStr(inputPathCStr);
    std::string outputPathStr(outputPathCStr);

    size_t lastSlash = outputPathStr.find_last_of("/");
    if (lastSlash != std::string::npos) {
        std::string dirPath = outputPathStr.substr(0, lastSlash);
        std::string mkdirCmd = "mkdir -p " + dirPath;
        system(mkdirCmd.c_str());
    }

    bool result = convertVocabulary(inputPathStr, outputPathStr);

    env->ReleaseStringUTFChars(inputPath, inputPathCStr);
    env->ReleaseStringUTFChars(outputPath, outputPathCStr);

    return result ? JNI_TRUE : JNI_FALSE;
}

// 初始化 SLAM 系统
extern "C" JNIEXPORT jboolean JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_initializeSLAM(
        JNIEnv* env,
        jobject /* this */,
        jstring vocPath,
        jstring configPath) {

    try {
        int availMem = getAvailableMemory();
        LOGI("Available memory: %d MB", availMem);

        if (availMem < 200) {
            LOGE("Not enough memory to initialize SLAM (available: %d MB)", availMem);
            return JNI_FALSE;
        }

        if (SLAM != nullptr) {
            LOGI("SLAM already initialized, shutting down existing instance");
            try {
                SLAM->Shutdown();
                delete SLAM;
                SLAM = nullptr;
            } catch (const std::exception& e) {
                LOGE("Error shutting down existing SLAM: %s", e.what());
            }
        }

        const char* vocPathCStr = env->GetStringUTFChars(vocPath, nullptr);
        const char* configPathCStr = env->GetStringUTFChars(configPath, nullptr);

        if (!vocPathCStr || !configPathCStr) {
            LOGE("Failed to get path strings from Java");
            if (vocPathCStr) env->ReleaseStringUTFChars(vocPath, vocPathCStr);
            if (configPathCStr) env->ReleaseStringUTFChars(configPath, configPathCStr);
            return JNI_FALSE;
        }

        std::string vocPathStr(vocPathCStr);
        std::string configPathStr(configPathCStr);

        LOGI("Checking SLAM initialization paths: voc=[%s], config=[%s]",
             vocPathStr.c_str(), configPathStr.c_str());

        bool vocExists = fileExists(vocPathStr);
        bool configExists = fileExists(configPathStr);

        if (!vocExists) {
            LOGE("Vocabulary file not found: %s", vocPathStr.c_str());
            std::string textVocPath = vocPathStr.substr(0, vocPathStr.find_last_of(".")) + ".txt";
            if (fileExists(textVocPath)) {
                LOGI("Found text vocabulary instead: %s", textVocPath.c_str());
                vocPathStr = textVocPath;
                vocExists = true;
            } else {
                LOGE("Neither binary nor text vocabulary found");
                env->ReleaseStringUTFChars(vocPath, vocPathCStr);
                env->ReleaseStringUTFChars(configPath, configPathCStr);
                return JNI_FALSE;
            }
        }

        if (!configExists) {
            LOGE("Config file not found: %s", configPathStr.c_str());
            env->ReleaseStringUTFChars(vocPath, vocPathCStr);
            env->ReleaseStringUTFChars(configPath, configPathCStr);
            return JNI_FALSE;
        }

        t0 = std::chrono::steady_clock::now();
        frameCount = 0;

        LOGI("Creating SLAM system in STEREO mode...");
        SLAM = new ORB_SLAM3::System(vocPathStr, configPathStr, ORB_SLAM3::System::STEREO, false);
        LOGI("SLAM initialization successful");

        env->ReleaseStringUTFChars(vocPath, vocPathCStr);
        env->ReleaseStringUTFChars(configPath, configPathCStr);
        return JNI_TRUE;
    }
    catch (const std::exception& e) {
        LOGE("Exception during SLAM initialization: %s", e.what());
        return JNI_FALSE;
    }
    catch (...) {
        LOGE("Unknown exception during SLAM initialization");
        return JNI_FALSE;
    }
}

// 关闭 SLAM 系统并保存轨迹
extern "C" JNIEXPORT jboolean JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_shutdownSLAM(
        JNIEnv* env,
        jobject /* this */,
        jstring trajectoryPath) {

    if (SLAM == nullptr) {
        LOGE("SLAM not initialized");
        return JNI_FALSE;
    }

    const char* trajectoryPathCStr = env->GetStringUTFChars(trajectoryPath, nullptr);
    if (!trajectoryPathCStr) {
        LOGE("Failed to get trajectory path");
        return JNI_FALSE;
    }

    std::string trajectoryPathStr(trajectoryPathCStr);

    try {
        LOGI("Shutting down SLAM and saving trajectory to: %s", trajectoryPathStr.c_str());

        size_t lastSlash = trajectoryPathStr.find_last_of("/");
        if (lastSlash != std::string::npos) {
            std::string dirPath = trajectoryPathStr.substr(0, lastSlash);
            std::string mkdirCmd = "mkdir -p " + dirPath;
            system(mkdirCmd.c_str());
        }

        SLAM->Shutdown();
        SLAM->SaveKeyFrameTrajectoryTUM(trajectoryPathStr);
        delete SLAM;
        SLAM = nullptr;

        // ❌ 删除GridMapManager清理代码
        // GridMapManager::getInstance().cleanupBuilder();

        env->ReleaseStringUTFChars(trajectoryPath, trajectoryPathCStr);
        return JNI_TRUE;
    } catch (const std::exception& e) {
        LOGE("Exception during SLAM shutdown: %s", e.what());
        env->ReleaseStringUTFChars(trajectoryPath, trajectoryPathCStr);
        return JNI_FALSE;
    } catch (...) {
        LOGE("Unknown exception during SLAM shutdown");
        env->ReleaseStringUTFChars(trajectoryPath, trajectoryPathCStr);
        return JNI_FALSE;
    }
}

// 获取SLAM跟踪状态
extern "C" JNIEXPORT jint JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_getSLAMTrackingState(
        JNIEnv* env,
        jobject /* this */) {

    if (SLAM == nullptr) {
        return -2;
    }

    try {
        return SLAM->GetTrackingState();
    } catch (const std::exception& e) {
        LOGE("Exception getting tracking state: %s", e.what());
        return -3;
    }
}

// ✅ 添加：检查是否有新关键帧
static int lastKeyFrameCount = 0;  // 静态变量记录上次关键帧数量

extern "C" JNIEXPORT jboolean JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_isNewKeyFrame(
        JNIEnv *env, jobject /* this */) {

    if (!SLAM || !SLAM->mpTracker || !SLAM->mpTracker->mpAtlas) {
        return JNI_FALSE;
    }

    try {
        // 获取所有有效关键帧
        std::vector<ORB_SLAM3::KeyFrame*> vpKFs = SLAM->mpTracker->mpAtlas->GetAllKeyFrames();
        int currentKeyFrameCount = 0;

        // 统计有效关键帧数量
        for (auto pKF : vpKFs) {
            if (pKF && !pKF->isBad()) {
                currentKeyFrameCount++;
            }
        }

        bool hasNewKeyFrame = (currentKeyFrameCount > lastKeyFrameCount);

        if (hasNewKeyFrame) {
            LOGI("Detected new keyframe: count changed from %d to %d",
                 lastKeyFrameCount, currentKeyFrameCount);
            lastKeyFrameCount = currentKeyFrameCount;
            return JNI_TRUE;
        }

        return JNI_FALSE;

    } catch (const std::exception& e) {
        LOGE("Exception checking new keyframe: %s", e.what());
        return JNI_FALSE;
    }
}



extern "C" JNIEXPORT jfloatArray JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_getLocalPointCloud(
        JNIEnv* env,
        jobject /* this */,
        jlong leftMatAddr) {

    if (!SLAM || !SLAM->mpTracker || !SLAM->mpTracker->mpAtlas) {
        LOGE("SLAM or Tracker is not initialized properly");
        return nullptr;
    }

    try {
        // 获取所有关键帧
        std::vector<ORB_SLAM3::KeyFrame*> keyFrames = SLAM->mpTracker->mpAtlas->GetAllKeyFrames();
        if (keyFrames.empty()) {
            LOGE("No keyframes available");
            return nullptr;
        }

        // 找到最新的关键帧
        ORB_SLAM3::KeyFrame* latestKF = nullptr;
        long maxId = -1;
        for (auto kf : keyFrames) {
            if (kf && !kf->isBad() && (long)kf->mnId > maxId) {
                maxId = (long)kf->mnId;
                latestKF = kf;
            }
        }

        if (!latestKF) {
            LOGE("No valid keyframe found");
            return nullptr;
        }

        // 获取地图点
        std::set<ORB_SLAM3::MapPoint*> mapPoints = latestKF->GetMapPoints();
        std::vector<float> pointData;
        pointData.reserve(mapPoints.size() * 6); // 每个点3个坐标和3个颜色分量

        // 将左图像转换为cv::Mat*
        cv::Mat* pLeftMat = (cv::Mat*)leftMatAddr;
        if (!pLeftMat || pLeftMat->empty()) {
            LOGE("Left image is null or empty");
            return nullptr;
        }

        for (auto mp : mapPoints) {
            if (!mp || mp->isBad()) continue;

            Eigen::Vector3f P3Dw = mp->GetWorldPos();
            float x = P3Dw.z();
            float y = -P3Dw.x();
            float z = -P3Dw.y();

            float u, v;
            cv::Point2f kp;
            if (latestKF->ProjectPointDistort(mp, kp, u, v)) {
                // 提取颜色信息
                if (u >= 0 && u < pLeftMat->cols && v >= 0 && v < pLeftMat->rows) {
                    cv::Vec3b color = pLeftMat->at<cv::Vec3b>(static_cast<int>(v), static_cast<int>(u)); // BGR
                    pointData.push_back(x);
                    pointData.push_back(y);
                    pointData.push_back(z);
                    pointData.push_back(color[2] / 255.0f); // R
                    pointData.push_back(color[1] / 255.0f); // G
                    pointData.push_back(color[0] / 255.0f); // B
                } else {
                    // 使用默认颜色（黑色）
                    pointData.push_back(x);
                    pointData.push_back(y);
                    pointData.push_back(z);
                    pointData.push_back(0.0f); // R
                    pointData.push_back(0.0f); // G
                    pointData.push_back(0.0f); // B
                }
            }
        }

        // 如果没有有效的点云数据
        if (pointData.empty()) {
            LOGE("Point data is empty after processing");
            return env->NewFloatArray(0); // 返回0长度数组
        }

        // 创建JNI float 数组并填充
        jfloatArray resultArray = env->NewFloatArray(pointData.size());
        env->SetFloatArrayRegion(resultArray, 0, pointData.size(), pointData.data());
        return resultArray;

    } catch (const std::exception& e) {
        LOGE("获取局部点云时出错: %s", e.what());
        return nullptr;
    }
}


// 保存地图到 .osa 文件
extern "C" JNIEXPORT jboolean JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_saveMapOSA(
        JNIEnv *env, jobject /* this */, jstring mapPath_) {
    if (!SLAM) {
        LOGE("SLAM instance not initialized, cannot save map.");
        return JNI_FALSE;
    }
    const char *mapPath = env->GetStringUTFChars(mapPath_, 0);
    bool res = false;
    try {
        LOGI("Saving ORB-SLAM3 map (.osa) to: %s", mapPath);
        res = SLAM->SaveMap(std::string(mapPath));    // ORB-SLAM3 System方法
    } catch (const std::exception &e) {
        LOGE("Exception during map saving: %s", e.what());
        res = false;
    }
    env->ReleaseStringUTFChars(mapPath_, mapPath);
    return res ? JNI_TRUE : JNI_FALSE;
}

// 添加从SLAM系统获取深度图的函数（更准确的方法）
extern "C" JNIEXPORT jlong JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_generateDepthMapFromSLAM(
        JNIEnv *env,
        jobject /* this */,
        jlong leftMatAddr) {

    if (!SLAM || !SLAM->mpTracker || !SLAM->mpTracker->mpAtlas) {
        LOGE("SLAM not initialized for depth map generation");
        return 0;
    }

    cv::Mat *pLeftMat = (cv::Mat*)leftMatAddr;
    if (!pLeftMat || pLeftMat->empty()) {
        LOGE("Input left image is null or empty");
        return 0;
    }

    try {
        // 创建深度图
        cv::Mat* depthMat = new cv::Mat(pLeftMat->rows, pLeftMat->cols, CV_32F, cv::Scalar(0.0f));

        // 获取当前帧的地图点和对应的关键点
        const std::vector<ORB_SLAM3::MapPoint*> vpMPs = SLAM->GetTrackedMapPoints();
        const std::vector<cv::KeyPoint> vKPs = SLAM->GetTrackedKeyPointsUn();

        if (vpMPs.empty() || vKPs.empty()) {
            LOGD("No tracked map points or keypoints available");
            delete depthMat;
            return 0;
        }

        if (vpMPs.size() != vKPs.size()) {
            LOGE("Map points and keypoints size mismatch: %d vs %d",
                 (int)vpMPs.size(), (int)vKPs.size());
            delete depthMat;
            return 0;
        }

        // 获取当前相机位姿
        Sophus::SE3f Tcw;
        try {
            Tcw = SLAM->mpTracker->mCurrentFrame.GetPose();
        } catch (const std::exception& e) {
            LOGE("Failed to get current frame pose: %s", e.what());
            delete depthMat;
            return 0;
        }

        int validDepthCount = 0;

        // 将地图点投影到图像平面并计算深度
        for (size_t i = 0; i < vpMPs.size(); i++) {
            if (!vpMPs[i] || vpMPs[i]->isBad()) continue;

            // 获取地图点在世界坐标系中的位置
            Eigen::Vector3f P3Dw = vpMPs[i]->GetWorldPos();

            // 转换到相机坐标系
            Eigen::Vector3f P3Dc = Tcw * P3Dw;

            // 深度值（Z坐标）
            float depth = P3Dc.z();

            // 只保留有效深度值
            if (depth > 0.1f && depth < 50.0f) { // 限制深度范围：0.1m到50m
                // 获取对应的关键点坐标
                cv::Point2f pt = vKPs[i].pt;
                int x = (int)std::round(pt.x);
                int y = (int)std::round(pt.y);

                // 确保坐标在图像范围内
                if (x >= 0 && x < depthMat->cols && y >= 0 && y < depthMat->rows) {
                    depthMat->at<float>(y, x) = depth;
                    validDepthCount++;
                }
            }
        }

        if (validDepthCount == 0) {
            LOGD("No valid depth values found");
            delete depthMat;
            return 0;
        }

        // 对深度图进行简单的形态学处理，填充小的空洞
        cv::Mat mask = (*depthMat) > 0;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 1);

        // 使用最近邻插值填充小的空洞
        cv::Mat dilated;
        cv::dilate(*depthMat, dilated, kernel, cv::Point(-1, -1), 1);
        dilated.copyTo(*depthMat, mask);

        LOGI("Generated SLAM depth map: %dx%d, valid points: %d/%d",
             depthMat->cols, depthMat->rows, validDepthCount, (int)vpMPs.size());

        return (jlong)depthMat;

    } catch (const std::exception& e) {
        LOGE("Exception generating SLAM depth map: %s", e.what());
        return 0;
    }
}

// 添加释放深度图内存的函数
extern "C" JNIEXPORT void JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_releaseDepthMap(
        JNIEnv *env,
        jobject /* this */,
        jlong depthMatAddr) {

    if (depthMatAddr != 0) {
        cv::Mat* depthMat = (cv::Mat*)depthMatAddr;
        delete depthMat;
    }
}

// 添加获取深度数据的JNI方法
extern "C" JNIEXPORT jfloatArray JNICALL
Java_com_vslam_orbslam3_vslamactivity_RosConnection_getDepthDataFromNative(
        JNIEnv *env,
        jobject /* this */,
        jlong depthMatAddr) {

    if (depthMatAddr == 0) {
        return nullptr;
    }

    cv::Mat* depthMat = (cv::Mat*)depthMatAddr;
    if (depthMat->empty()) {
        return nullptr;
    }

    try {
        // 确保是32位浮点数格式
        cv::Mat depthFloat;
        if (depthMat->type() != CV_32F) {
            depthMat->convertTo(depthFloat, CV_32F);
        } else {
            depthFloat = *depthMat;
        }

        // 创建float数组
        int totalSize = depthFloat.rows * depthFloat.cols;
        jfloatArray result = env->NewFloatArray(totalSize);

        if (result == nullptr) {
            LOGE("Failed to create float array for depth data");
            return nullptr;
        }

        // 复制数据
        float* data = (float*)depthFloat.data;
        env->SetFloatArrayRegion(result, 0, totalSize, data);

        return result;

    } catch (const std::exception& e) {
        LOGE("Exception getting depth data: %s", e.what());
        return nullptr;
    }
}

// 添加获取深度图尺寸的JNI方法
extern "C" JNIEXPORT jintArray JNICALL
Java_com_vslam_orbslam3_vslamactivity_RosConnection_getDepthDimensionsFromNative(
        JNIEnv *env,
        jobject /* this */,
        jlong depthMatAddr) {

    if (depthMatAddr == 0) {
        return nullptr;
    }

    cv::Mat* depthMat = (cv::Mat*)depthMatAddr;
    if (depthMat->empty()) {
        return nullptr;
    }

    try {
        jintArray result = env->NewIntArray(2);
        if (result == nullptr) {
            return nullptr;
        }

        int dimensions[2] = {depthMat->cols, depthMat->rows};
        env->SetIntArrayRegion(result, 0, 2, dimensions);

        return result;

    } catch (const std::exception& e) {
        LOGE("Exception getting depth dimensions: %s", e.what());
        return nullptr;
    }
}




// ✅ 添加：获取当前关键帧数据
extern "C" JNIEXPORT jfloatArray JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_getCurrentKeyFrameData(
        JNIEnv *env, jobject /* this */) {

    if (!SLAM || !SLAM->mpTracker || !SLAM->mpTracker->mpAtlas) {
        LOGE("SLAM not initialized when getting keyframe data");
        return nullptr;
    }

    try {
        // 获取所有关键帧
        std::vector<ORB_SLAM3::KeyFrame*> vpKFs = SLAM->mpTracker->mpAtlas->GetAllKeyFrames();

        if (vpKFs.empty()) {
            LOGD("No keyframes available");
            return nullptr;
        }

        // 找到最新的有效关键帧（按ID排序）
        ORB_SLAM3::KeyFrame* pLatestKF = nullptr;
        long maxId = -1;

        for (auto pKF : vpKFs) {
            if (pKF && !pKF->isBad() && (long)pKF->mnId > maxId) {
                maxId = (long)pKF->mnId;
                pLatestKF = pKF;
            }
        }

        if (!pLatestKF) {
            LOGD("No valid keyframe found");
            return nullptr;
        }

        LOGI("Using latest keyframe with ID: %ld", (long)pLatestKF->mnId);

        // 获取关键帧位姿
        Sophus::SE3f Twc = pLatestKF->GetPoseInverse();
        Eigen::Matrix3f Rwc = Twc.rotationMatrix();
        Eigen::Vector3f twc = Twc.translation();
        Eigen::Quaternionf q(Rwc);

        // 获取关键帧观测到的地图点
        std::set<ORB_SLAM3::MapPoint*> mapPoints = pLatestKF->GetMapPoints();

        std::vector<float> validPoints;
        validPoints.reserve(mapPoints.size() * 3);

        int validPointsCount = 0;
        for (auto mp : mapPoints) {
            if (!mp || mp->isBad()) continue;

            Eigen::Vector3f P3Dw = mp->GetWorldPos();

            // 应用C++ ROS版本的坐标转换（无IMU情况）
            validPoints.push_back(P3Dw.z());    // Z → X
            validPoints.push_back(-P3Dw.x());   // -X → Y
            validPoints.push_back(-P3Dw.y());   // -Y → Z
            validPointsCount++;
        }

        if (validPointsCount == 0) {
            LOGD("No valid map points in keyframe %ld", (long)pLatestKF->mnId);
            return nullptr;
        }

        // 构造返回数据：[kf_pose(7个float), map_points(3n个float)]
        std::vector<float> result;
        result.reserve(7 + validPoints.size());

        // 添加关键帧位姿（应用坐标转换）
        result.push_back(twc.z());    // Z → X
        result.push_back(-twc.x());   // -X → Y
        result.push_back(-twc.y());   // -Y → Z
        result.push_back(q.z());      // qz → qx
        result.push_back(-q.x());     // -qx → qy
        result.push_back(-q.y());     // -qy → qz
        result.push_back(q.w());      // qw → qw

        // 添加地图点
        result.insert(result.end(), validPoints.begin(), validPoints.end());

        LOGI("Keyframe %ld has %d valid points", (long)pLatestKF->mnId, validPointsCount);

        jfloatArray resultArray = env->NewFloatArray(result.size());
        if (resultArray == nullptr) {
            LOGE("Failed to create result array for keyframe data");
            return nullptr;
        }

        env->SetFloatArrayRegion(resultArray, 0, result.size(), result.data());
        return resultArray;

    } catch (const std::exception& e) {
        LOGE("Exception getting current keyframe data: %s", e.what());
        return nullptr;
    }
}

// 处理并跟踪双目图像帧
// 处理并跟踪双目图像帧（修改版 - 7元素向量格式，推荐）
extern "C" JNIEXPORT jfloatArray JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_processStereoFrame(
        JNIEnv *env,
        jobject /* this */,
        jlong leftMatAddr,
        jlong rightMatAddr,
        jdouble timestamp) {

    if (!SLAM) {
        LOGE("SLAM not initialized for stereo frame processing");
        return nullptr;
    }

    cv::Mat *pLeftMat = (cv::Mat*)leftMatAddr;
    cv::Mat *pRightMat = (cv::Mat*)rightMatAddr;

    if (!pLeftMat || pLeftMat->empty() || !pRightMat || pRightMat->empty()) {
        LOGE("Input images are null or empty");
        return nullptr;
    }

    double frameTimestamp = timestamp;
    if (timestamp <= 0) {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        frameTimestamp = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
    }

    std::vector<float> poseData;

    try {
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

        // 获取 Tcw (相机到世界变换)
        Sophus::SE3f Tcw_SE3f = SLAM->TrackStereo(*pLeftMat, *pRightMat, frameTimestamp, vImuMeas, std::to_string(frameCount));
        frameCount++;

        // ✅ 转换为 Twc (相机在世界坐标系中的位姿)
        Sophus::SE3f Twc_SE3f = Tcw_SE3f.inverse();
        Eigen::Matrix3f Rwc = Twc_SE3f.rotationMatrix();
        Eigen::Vector3f twc = Twc_SE3f.translation();
        Eigen::Quaternionf q(Rwc);

        // ✅ 应用坐标系转换 (ORB-SLAM3 → ROS) - 与其他函数保持一致
        poseData.reserve(7);
        poseData.push_back(twc.z());    // Z → X
        poseData.push_back(-twc.x());   // -X → Y
        poseData.push_back(-twc.y());   // -Y → Z
        poseData.push_back(q.z());      // qz → qx
        poseData.push_back(-q.x());     // -qx → qy
        poseData.push_back(-q.y());     // -qy → qz
        poseData.push_back(q.w());      // qw → qw

    } catch (const std::exception& e) {
        LOGE("Exception during stereo frame tracking: %s", e.what());
        return nullptr;
    } catch (...) {
        LOGE("Unknown exception during stereo frame tracking");
        return nullptr;
    }

    int state = SLAM->GetTrackingState();

    // 在左图像上显示跟踪状态
    switch (state) {
        case -1:
            cv::putText(*pLeftMat, "SYSTEM NOT READY", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
            break;
        case 0:
            cv::putText(*pLeftMat, "NO IMAGES YET", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
            break;
        case 1:
            cv::putText(*pLeftMat, "SLAM NOT INITIALIZED", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
            break;
        case 2:
            cv::putText(*pLeftMat, "SLAM ON", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
            break;
        case 3:
            cv::putText(*pLeftMat, "SLAM LOST", cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
            break;
        default:
            break;
    }

    if (!poseData.empty() && state == 2) {
        const std::vector<ORB_SLAM3::MapPoint*> vpMPs = SLAM->GetTrackedMapPoints();
        const std::vector<cv::KeyPoint> vKPs = SLAM->GetTrackedKeyPointsUn();

        for (size_t i = 0; i < vpMPs.size(); i++) {
            if (vpMPs[i]) {
                cv::circle(*pLeftMat, vKPs[i].pt, 2, cv::Scalar(0, 255, 0), -1);
            }
        }

        std::string infoText = "Frame: " + std::to_string(frameCount) +
                               " | Points: " + std::to_string(vpMPs.size()) + " | ROS Coord";
        cv::putText(*pLeftMat, infoText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                    0.5, cv::Scalar(0, 255, 0), 1);
    }

    if (!poseData.empty()) {
        jfloatArray resultArray = env->NewFloatArray(7);
        if (resultArray == nullptr) {
            LOGE("Failed to create result array");
            return nullptr;
        }

        env->SetFloatArrayRegion(resultArray, 0, 7, poseData.data());
        return resultArray;
    }

    return nullptr;
}

// 获取当前跟踪到的地图点
extern "C" JNIEXPORT jfloatArray JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_getTrackedMapPoints(
        JNIEnv *env, jobject /* this */) {

    if (!SLAM) {
        LOGE("SLAM not initialized when trying to get tracked map points");
        return nullptr;
    }

    std::vector<ORB_SLAM3::MapPoint*> mapPoints = SLAM->GetTrackedMapPoints();

    if (mapPoints.empty()) {
        LOGD("No tracked map points available");
        return nullptr;
    }

    std::vector<float> pointData;
    pointData.reserve(mapPoints.size() * 3);

    int validPointsCount = 0;
    for (const auto &mp : mapPoints) {
        if (!mp || mp->isBad()) continue;

        Eigen::Vector3f worldPos = mp->GetWorldPos();
        pointData.push_back(worldPos.x());
        pointData.push_back(worldPos.y());
        pointData.push_back(worldPos.z());
        validPointsCount++;
    }

    if (validPointsCount == 0) {
        LOGD("No valid map points found");
        return nullptr;
    }

    //LOGI("Found %d valid map points", validPointsCount);

    jfloatArray result = env->NewFloatArray(pointData.size());
    if (result == nullptr) {
        LOGE("Failed to create Java float array for map points");
        return nullptr;
    }

    env->SetFloatArrayRegion(result, 0, pointData.size(), pointData.data());
    return result;
}

// 获取所有关键帧及其地图点数据
extern "C" JNIEXPORT jfloatArray JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_getAllKeyframeData(
        JNIEnv *env, jobject /* this */) {

    if (!SLAM) {
        LOGE("SLAM not initialized when trying to get keyframe data");
        return nullptr;
    }

    std::vector<ORB_SLAM3::KeyFrame*> keyFrames;

    if (SLAM->mpTracker && SLAM->mpTracker->mpAtlas) {
        keyFrames = SLAM->mpTracker->mpAtlas->GetAllKeyFrames();
    } else {
        LOGD("Atlas not available");
        return nullptr;
    }

    if (keyFrames.empty()) {
        LOGD("No keyframes available");
        return nullptr;
    }

    LOGI("Found %d keyframes", (int)keyFrames.size());

    std::vector<float> data;
    data.reserve(keyFrames.size() * (7 + 1 + 100 * 3) + 1);

    data.push_back(static_cast<float>(keyFrames.size()));

    std::sort(keyFrames.begin(), keyFrames.end(), [](const ORB_SLAM3::KeyFrame* a, const ORB_SLAM3::KeyFrame* b) {
        return a->mnId < b->mnId;
    });

    int validKfCount = 0;

    for (const auto &kf : keyFrames) {
        if (!kf || kf->isBad()) continue;

        validKfCount++;

        Sophus::SE3f Twc = kf->GetPoseInverse();
        Eigen::Matrix3f Rwc = Twc.rotationMatrix();
        Eigen::Vector3f twc = Twc.translation();
        Eigen::Quaternionf q(Rwc);

        // 坐标转换——对齐C++ ROS非IMU版本:
        data.push_back(twc.z());    // z -> x
        data.push_back(-twc.x());   // -x -> y
        data.push_back(-twc.y());   // -y -> z

        data.push_back(q.z());      // qz -> qx
        data.push_back(-q.x());     // -qx -> qy
        data.push_back(-q.y());     // -qy -> qz
        data.push_back(q.w());      // qw -> qw

        std::set<ORB_SLAM3::MapPoint*> mapPoints = kf->GetMapPoints();

        std::vector<Eigen::Vector3f> validPoints;
        validPoints.reserve(mapPoints.size());

        for (auto mp : mapPoints) {
            if (!mp || mp->isBad()) continue;
            Eigen::Vector3f pt = mp->GetWorldPos();
            validPoints.push_back(pt);
        }

        data.push_back(static_cast<float>(validPoints.size()));

        LOGD("Keyframe %d has %d valid points", (int)kf->mnId, (int)validPoints.size());

        for (const auto &pt : validPoints) {
            // 坐标转换——对齐C++ ROS非IMU版本:
            data.push_back(pt.z());    // z -> x
            data.push_back(-pt.x());   // -x -> y
            data.push_back(-pt.y());   // -y -> z
        }
    }

    if (!data.empty()) {
        data[0] = static_cast<float>(validKfCount);
    }

    if (data.size() <= 1) {
        LOGD("No valid keyframe data available");
        return nullptr;
    }

    LOGI("Exporting %d total data values for processing", (int)data.size());

    jfloatArray result = env->NewFloatArray(data.size());
    if (result == nullptr) {
        LOGE("Failed to create Java float array for keyframe data");
        return nullptr;
    }

    env->SetFloatArrayRegion(result, 0, data.size(), data.data());
    return result;
}



// 检查是否检测到闭环
extern "C" JNIEXPORT jboolean JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_checkLoopClosure(
        JNIEnv *env, jobject /* this */) {

    if (!SLAM) {
        LOGE("SLAM not initialized when checking loop closure");
        return JNI_FALSE;
    }

    bool mapChanged = false;

    try {
        mapChanged = SLAM->MapChanged();
    } catch (const std::exception& e) {
        LOGE("Exception when checking map changes: %s", e.what());
        return JNI_FALSE;
    } catch (...) {
        LOGE("Unknown exception when checking map changes");
        return JNI_FALSE;
    }

    if (mapChanged) {
        LOGI("Map has changed, possibly due to loop closure");

    }

    return mapChanged ? JNI_TRUE : JNI_FALSE;
}

// 修正的A*算法JNI函数 - 匹配PathPlanningManager
extern "C" JNIEXPORT jintArray JNICALL
Java_com_vslam_orbslam3_vslamactivity_PathPlanningManager_nativeFindPath(
        JNIEnv *env,
        jobject /* this */,
        jintArray mapData,
        jint width,
        jint height,
        jint startX,
        jint startY,
        jint endX,
        jint endY,
        jdouble weightA,
        jdouble weightB,
        jstring distance) {

    LOGI("Starting A* pathfinding: start(%d,%d) -> end(%d,%d)", startX, startY, endX, endY);

    try {
        // 获取地图数据
        jint* mapArray = env->GetIntArrayElements(mapData, nullptr);
        jsize mapSize = env->GetArrayLength(mapData);

        if (mapArray == nullptr) {
            LOGE("Failed to get map array elements");
            return nullptr;
        }

        std::vector<int> mapVector(mapArray, mapArray + mapSize);

        // 获取距离类型字符串
        const char* distanceStr = env->GetStringUTFChars(distance, nullptr);
        if (distanceStr == nullptr) {
            LOGE("Failed to get distance string");
            env->ReleaseIntArrayElements(mapData, mapArray, 0);
            return nullptr;
        }

        std::string distanceType(distanceStr);

        // 验证输入参数
        if (startX < 0 || startX >= width || startY < 0 || startY >= height ||
            endX < 0 || endX >= width || endY < 0 || endY >= height) {
            LOGE("Invalid start or end coordinates");
            env->ReleaseIntArrayElements(mapData, mapArray, 0);
            env->ReleaseStringUTFChars(distance, distanceStr);
            return nullptr;
        }

        // 使用完整的命名空间创建A*对象
        ASTAR::CAstar astar(startX, startY, endX, endY, weightA, weightB,
                            ASTAR::CAstar::NOFINDPATHPOINT, distanceType);

        // 初始化地图
        astar.InitMap(mapVector, width, height);

        // 执行路径规划
        std::vector<std::pair<int, int>> path = astar.PathPoint();

        // 释放资源
        env->ReleaseIntArrayElements(mapData, mapArray, 0);
        env->ReleaseStringUTFChars(distance, distanceStr);

        // 检查是否找到路径
        if (astar.m_noPathFlag == ASTAR::CAstar::NOFINDPATHPOINT) {
            LOGE("No path found by A* algorithm");
            return nullptr;
        }

        // 将路径转换为Java数组
        if (path.empty()) {
            LOGE("A* returned empty path");
            return nullptr;
        }

        LOGI("A* found path with %d points", (int)path.size());

        // 创建返回数组
        jintArray result = env->NewIntArray(path.size() * 2);
        if (result == nullptr) {
            LOGE("Failed to create result array");
            return nullptr;
        }

        // 填充路径数据
        std::vector<jint> pathArray;
        pathArray.reserve(path.size() * 2);

        for (size_t i = 0; i < path.size(); ++i) {
            pathArray.push_back(path[i].first);   // x坐标
            pathArray.push_back(path[i].second);  // y坐标
        }

        env->SetIntArrayRegion(result, 0, pathArray.size(), pathArray.data());

        LOGI("Successfully created path array with %d elements", (int)pathArray.size());
        return result;

    } catch (const std::exception& e) {
        LOGE("Exception in nativeFindPath: %s", e.what());
        return nullptr;
    } catch (...) {
        LOGE("Unknown exception in nativeFindPath");
        return nullptr;
    }
}



// 检查文件有效性
extern "C" JNIEXPORT jboolean JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_checkFileValidity(
        JNIEnv* env,
        jobject /* this */,
        jstring filePath) {

    const char* filePathCStr = env->GetStringUTFChars(filePath, nullptr);
    if (!filePathCStr) {
        return JNI_FALSE;
    }

    std::string filePathStr(filePathCStr);
    LOGI("Checking file validity: %s", filePathStr.c_str());

    bool exists = fileExists(filePathStr);

    if (exists && (filePathStr.find("voc") != std::string::npos ||
                   filePathStr.find("Voc") != std::string::npos)) {

        FILE* f = fopen(filePathStr.c_str(), "rb");
        if (!f) {
            LOGE("Cannot open file for reading");
            env->ReleaseStringUTFChars(filePath, filePathCStr);
            return JNI_FALSE;
        }

        char header[8] = {0};
        size_t read = fread(header, 1, 8, f);
        fclose(f);

        if (read < 8) {
            LOGE("File too small to be a valid vocabulary");
            env->ReleaseStringUTFChars(filePath, filePathCStr);
            return JNI_FALSE;
        }

        long fileSize = 0;
        f = fopen(filePathStr.c_str(), "rb");
        if (f) {
            fseek(f, 0, SEEK_END);
            fileSize = ftell(f);
            fclose(f);

            double fileSizeMB = fileSize / (1024.0 * 1024.0);
            LOGI("File size: %.2f MB", fileSizeMB);

            if (fileSizeMB < 1.0) {
                LOGE("Vocabulary file too small (%.2f MB)", fileSizeMB);
                env->ReleaseStringUTFChars(filePath, filePathCStr);
                return JNI_FALSE;
            }
        }
    }

    env->ReleaseStringUTFChars(filePath, filePathCStr);
    return exists ? JNI_TRUE : JNI_FALSE;
}

// 生成小词汇表
extern "C" JNIEXPORT jboolean JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_generateSmallVocabulary(
        JNIEnv* env,
        jobject /* this */,
        jstring inputPath,
        jstring outputPath) {

    const char* inputPathCStr = env->GetStringUTFChars(inputPath, nullptr);
    const char* outputPathCStr = env->GetStringUTFChars(outputPath, nullptr);

    if (!inputPathCStr || !outputPathCStr) {
        LOGE("Failed to get path strings");
        if (inputPathCStr) env->ReleaseStringUTFChars(inputPath, inputPathCStr);
        if (outputPathCStr) env->ReleaseStringUTFChars(outputPath, outputPathCStr);
        return JNI_FALSE;
    }

    std::string inputPathStr(inputPathCStr);
    std::string outputPathStr(outputPathCStr);

    LOGI("Attempting to convert vocabulary from: %s to: %s", inputPathStr.c_str(), outputPathStr.c_str());

    if (!fileExists(inputPathStr)) {
        LOGE("Source vocabulary not found: %s", inputPathStr.c_str());
        env->ReleaseStringUTFChars(inputPath, inputPathCStr);
        env->ReleaseStringUTFChars(outputPath, outputPathCStr);
        return JNI_FALSE;
    }

    if (fileExists(outputPathStr)) {
        LOGI("Target vocabulary already exists: %s", outputPathStr.c_str());
        env->ReleaseStringUTFChars(inputPath, inputPathCStr);
        env->ReleaseStringUTFChars(outputPath, outputPathCStr);
        return JNI_TRUE;
    }

    // 创建输出目录
    size_t lastSlash = outputPathStr.find_last_of("/");
    if (lastSlash != std::string::npos) {
        std::string dirPath = outputPathStr.substr(0, lastSlash);
        std::string mkdirCmd = "mkdir -p " + dirPath;
        system(mkdirCmd.c_str());
    }

    try {
        LOGI("Starting vocabulary conversion");
        bool result = false;

        // 对于文本格式转二进制格式的情况
        if (inputPathStr.find(".txt") != std::string::npos &&
            outputPathStr.find(".bin") != std::string::npos) {
            LOGI("Converting text to binary vocabulary");
            result = convertVocabulary(inputPathStr, outputPathStr);
        }
            // 如果两者都是二进制，尝试复制文件
        else if (inputPathStr.find(".bin") != std::string::npos &&
                 outputPathStr.find(".bin") != std::string::npos) {
            std::ifstream src(inputPathStr, std::ios::binary);
            std::ofstream dst(outputPathStr, std::ios::binary);
            dst << src.rdbuf();
            result = dst.good();
            LOGI("Binary file copy %s", result ? "succeeded" : "failed");
        }
        else {
            LOGI("Unsupported conversion format");
        }

        env->ReleaseStringUTFChars(inputPath, inputPathCStr);
        env->ReleaseStringUTFChars(outputPath, outputPathCStr);
        return result ? JNI_TRUE : JNI_FALSE;
    }
    catch (const std::exception& e) {
        LOGE("Exception during vocabulary conversion: %s", e.what());
        env->ReleaseStringUTFChars(inputPath, inputPathCStr);
        env->ReleaseStringUTFChars(outputPath, outputPathCStr);
        return JNI_FALSE;
    }
}

// 获取文件信息
extern "C" JNIEXPORT jstring JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_getFileInfo(
        JNIEnv* env,
        jobject /* this */,
        jstring filePath) {

    const char* filePathCStr = env->GetStringUTFChars(filePath, nullptr);
    if (!filePathCStr) {
        return env->NewStringUTF("Error: Could not get file path");
    }

    std::string filePathStr(filePathCStr);
    LOGI("Getting file info: %s", filePathStr.c_str());

    std::string result = "File: " + filePathStr + "\n";

    if (!fileExists(filePathStr)) {
        result += "Status: File does not exist";
        env->ReleaseStringUTFChars(filePath, filePathCStr);
        return env->NewStringUTF(result.c_str());
    }

    FILE* f = fopen(filePathStr.c_str(), "rb");
    if (f) {
        fseek(f, 0, SEEK_END);
        long fileSize = ftell(f);
        fseek(f, 0, SEEK_SET);

        double fileSizeMB = fileSize / (1024.0 * 1024.0);
        result += "Size: " + std::to_string(fileSize) + " bytes (" +
                  std::to_string(fileSizeMB) + " MB)\n";

        unsigned char header[16] = {0};
        size_t read = fread(header, 1, 16, f);
        fclose(f);

        result += "Read bytes: " + std::to_string(read) + "\n";

        std::stringstream ss;
        for (size_t i = 0; i < read && i < 16; i++) {
            ss << std::hex << std::setw(2) << std::setfill('0')
               << (int)header[i] << " ";
        }
        result += "Header: " + ss.str() + "\n";

        if (read > 0 && isprint(header[0])) {
            f = fopen(filePathStr.c_str(), "r");
            if (f) {
                char textBuf[101] = {0};
                fread(textBuf, 1, 100, f);
                fclose(f);
                result += "Content start: " + std::string(textBuf) + "...\n";
            }
        }
    } else {
        result += "Status: Could not open file for reading\n";
    }
    env->ReleaseStringUTFChars(filePath, filePathCStr);
    return env->NewStringUTF(result.c_str());
}


// 获取地图总数
extern "C" JNIEXPORT jint JNICALL
Java_com_vslam_orbslam3_vslamactivity_VslamActivity_getMapCount(JNIEnv *env, jobject) {
    if (!SLAM || !SLAM->mpTracker || !SLAM->mpTracker->mpAtlas) return 0;
    try {
        return SLAM->mpTracker->mpAtlas->GetAllMaps().size();
    } catch (...) {
        return 0;
    }
}
