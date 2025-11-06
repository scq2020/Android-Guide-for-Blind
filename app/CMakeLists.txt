# For more information about using CMake with Android Studio, read the  
# documentation: https://d.android.com/studio/projects/add-native-code.html  

# Sets the minimum version of CMake required to build the native library.  
cmake_minimum_required(VERSION 3.4.1)  

SET(CMAKE_BUILD_TYPE Release)  

# 打印当前的构建类型  
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")  
message(STATUS "Android ABI: ${ANDROID_ABI}")  

# 现代 CMake 方式设置 C++ 标准  
set(CMAKE_CXX_STANDARD 14)  
set(CMAKE_CXX_STANDARD_REQUIRED ON)  
set(CMAKE_CXX_EXTENSIONS OFF)  

# 完全禁用 OpenMP
add_definitions(-DNO_OPENMP)
string(REPLACE "-fopenmp" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
string(REPLACE "-fopenmp" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")

# 设置编译标志 - 避免重复
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti -fexceptions -Wno-deprecated-declarations")

message(STATUS "Building without OpenMP support")
message(STATUS "Final C++ Flags: ${CMAKE_CXX_FLAGS}")

find_library(log-lib log)

# 重新组织include目录，确保Eigen在OpenCV之前
include_directories(  
    # 首先包含Eigen
    ${PROJECT_SOURCE_DIR}/src/main/cpp/Eigen/eigen-3.4.0
    ${PROJECT_SOURCE_DIR}/src/main/cpp/Eigen/eigen-3.4.0/src
    
    # 然后是项目头文件
    ${PROJECT_SOURCE_DIR}/src/main/cpp  
    ${PROJECT_SOURCE_DIR}/src/main/cpp/compat  
    
    # ===== 新增：A*算法头文件目录 =====
    ${PROJECT_SOURCE_DIR}/src/main/cpp/astar
    
    # ORB-SLAM3相关
    ${PROJECT_SOURCE_DIR}/src/main/cpp/ORB/include  
    ${PROJECT_SOURCE_DIR}/src/main/cpp/ORB  
    ${PROJECT_SOURCE_DIR}/src/main/cpp/ORB/include/CameraModels  
    ${PROJECT_SOURCE_DIR}/src/main/cpp/ORB/Thirdparty  
    ${PROJECT_SOURCE_DIR}/src/main/cpp/ORB/Thirdparty/Sophus  
    ${PROJECT_SOURCE_DIR}/src/main/cpp/ORB/Thirdparty/Sophus/sophus  
    ${PROJECT_SOURCE_DIR}/src/main/cpp/stereo_image_proc
    ${PROJECT_SOURCE_DIR}/src/main/cpp/stereo_image_proc/include
    
    # 其他库
    ${PROJECT_SOURCE_DIR}/src/main/cpp/boost  
    ${PROJECT_SOURCE_DIR}/src/main/cpp/openssl/openssl-1.0.2s  
    ${PROJECT_SOURCE_DIR}/src/main/cpp/openssl/openssl-1.0.2s/openssl  
    
    # 最后是OpenCV (在Eigen之后)
    ${PROJECT_SOURCE_DIR}/src/main/cpp/opencv/opencv-4.5.5/include  
    ${PROJECT_SOURCE_DIR}/../opencv/native/jni/include  
    ${PROJECT_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/opencv/include
)  

# ===== 新增：A*算法静态库 =====
ADD_LIBRARY(astar STATIC
    src/main/cpp/astar/Astar.cpp
    # 如果有其他相关文件，在这里添加，例如：
    # src/main/cpp/astar/AstarNode.cpp
    # src/main/cpp/astar/PathUtils.cpp
    # src/main/cpp/astar/MapUtils.cpp
)

# g2o库
ADD_LIBRARY(g2o STATIC
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/batch_stats.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/cache.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/estimate_propagator.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/factory.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/hyper_graph.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/hyper_graph_action.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/jacobian_workspace.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/matrix_structure.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/optimizable_graph.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/optimization_algorithm.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/optimization_algorithm_dogleg.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/parameter.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/parameter_container.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/robust_kernel.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/solver.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/core/sparse_optimizer.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/stuff/os_specific.c
    src/main/cpp/ORB/Thirdparty/g2o/g2o/stuff/property.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/stuff/string_tools.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/stuff/timeutil.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/types/se3mat.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/types/types_sba.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp
    src/main/cpp/ORB/Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp
)  

# DBoW2库
ADD_LIBRARY(DBoW2 STATIC
    src/main/cpp/ORB/Thirdparty/DBoW2/DBoW2/BowVector.cpp  
    src/main/cpp/ORB/Thirdparty/DBoW2/DBoW2/FORB.cpp  
    src/main/cpp/ORB/Thirdparty/DBoW2/DBoW2/FeatureVector.cpp  
    src/main/cpp/ORB/Thirdparty/DBoW2/DBoW2/ScoringObject.cpp  
    src/main/cpp/ORB/Thirdparty/DBoW2/DUtils/Random.cpp  
    src/main/cpp/ORB/Thirdparty/DBoW2/DUtils/Timestamp.cpp  
)  

# ORB-SLAM3库
add_library(ORBSLAM3 STATIC
    src/main/cpp/ORB/src/Atlas.cc  
    src/main/cpp/ORB/src/CameraModels/KannalaBrandt8.cpp  
    src/main/cpp/ORB/src/CameraModels/Pinhole.cpp  
    src/main/cpp/ORB/src/Config.cc  
    src/main/cpp/ORB/src/Converter.cc  
    src/main/cpp/ORB/src/Frame.cc  
    src/main/cpp/ORB/src/FrameDrawer.cc  
    src/main/cpp/ORB/src/G2oTypes.cc  
    src/main/cpp/ORB/src/GeometricTools.cc  
    src/main/cpp/ORB/src/ImuTypes.cc  
    src/main/cpp/ORB/src/KeyFrame.cc  
    src/main/cpp/ORB/src/KeyFrameDatabase.cc  
    src/main/cpp/ORB/src/LocalMapping.cc  
    src/main/cpp/ORB/src/LoopClosing.cc  
    src/main/cpp/ORB/src/MLPnPsolver.cpp  
    src/main/cpp/ORB/src/Map.cc  
    src/main/cpp/ORB/src/MapPoint.cc  
    src/main/cpp/ORB/src/ORBextractor.cc  
    src/main/cpp/ORB/src/ORBmatcher.cc  
    src/main/cpp/ORB/src/OptimizableTypes.cpp  
    src/main/cpp/ORB/src/Optimizer.cc  
    src/main/cpp/ORB/src/Settings.cc  
    src/main/cpp/ORB/src/Sim3Solver.cc  
    src/main/cpp/ORB/src/System.cc  
    src/main/cpp/ORB/src/Tracking.cc  
    src/main/cpp/ORB/src/TwoViewReconstruction.cc  
    src/main/cpp/stereo_image_proc/src/processor.cpp  
)  

# 导入的静态库
add_library(boost_system STATIC IMPORTED)  
set_target_properties(boost_system PROPERTIES IMPORTED_LOCATION  
${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libboost_1_72_0/libboost_system.a)  

add_library(boost_atomic STATIC IMPORTED)  
set_target_properties(boost_atomic PROPERTIES IMPORTED_LOCATION  
${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libboost_1_72_0/libboost_atomic.a)  

add_library(boost_serialization STATIC IMPORTED)  
set_target_properties(boost_serialization PROPERTIES IMPORTED_LOCATION  
${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libboost_1_72_0/libboost_serialization.a)

add_library(ssl STATIC IMPORTED)
set_target_properties(ssl PROPERTIES IMPORTED_LOCATION
        ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopenssl-1.0.2s/libssl.a)

add_library(crypto STATIC IMPORTED)
set_target_properties(crypto PROPERTIES IMPORTED_LOCATION
        ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopenssl-1.0.2s/libcrypto.a)

# OpenCV库
add_library(libopencv_java4 SHARED IMPORTED)
set_target_properties(libopencv_java4 PROPERTIES IMPORTED_LOCATION
        ${CMAKE_SOURCE_DIR}/src/main/jniLibs/${ANDROID_ABI}/libopencv_java4.so)

# 设置ORBSLAM3的依赖
target_link_libraries(ORBSLAM3
        g2o
        DBoW2
        boost_system
        boost_serialization
        ssl
        crypto
        )

# 主要的native-lib
add_library(native-lib SHARED
    src/main/cpp/native-lib.cpp
    src/main/cpp/Plane.cpp  
    src/main/cpp/Frame.cpp  
    src/main/cpp/Marker.cpp  
    src/main/cpp/Process.cpp  
    src/main/cpp/Camera.cpp  
)  

# 链接native-lib的依赖 - 修改：添加astar库
target_link_libraries(native-lib
                    ${log-lib}
                    ORBSLAM3
                    astar                    # ===== 新增：链接A*算法库 =====
                    libopencv_java4
)

# 输出最终构建信息
message(STATUS "=== Build Configuration Summary ===")
message(STATUS "Android ABI: ${ANDROID_ABI}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "OpenMP Support: DISABLED")
message(STATUS "A* Algorithm: ENABLED")              # ===== 新增：A*算法状态 =====
message(STATUS "Final C++ Flags: ${CMAKE_CXX_FLAGS}")
message(STATUS "===================================")