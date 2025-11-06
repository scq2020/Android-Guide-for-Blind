// RosManager.java
package com.vslam.orbslam3.vslamactivity;

import android.content.Context;
import android.graphics.Color;
import android.graphics.drawable.GradientDrawable;
import android.os.Handler;
import android.util.Log;
import android.util.TypedValue;
import android.view.Gravity;
import android.view.ViewGroup;
import android.widget.FrameLayout;
import android.widget.TextView;

import org.opencv.core.Mat;
import java.util.List;
import java.util.ArrayList;

/**
 * ROS功能管理类，负责管理ROS连接、状态显示和数据发布
 */
public class RosManager {
    private static final String TAG = "RosManager";
    private static final String DEFAULT_ROS_MASTER_URI = "http://192.168.1.131:11311";

    private final VslamActivity activity;
    private RosConnection rosConnection;
    private TextView rosStatusText;
    private Handler statusUpdateHandler;
    private final GradientDrawable statusIndicator = new GradientDrawable();

    // ✅ 新增：点云发布器
    private LocalPointCloudPublisher localPointCloudPublisher;

    /**
     * 构造函数
     * @param activity VSLAM活动
     */
    public RosManager(VslamActivity activity) {
        this.activity = activity;
        initialize();
    }

    /**
     * 初始化ROS功能
     */
    private void initialize() {
        // 初始化ROS连接
        rosConnection = new RosConnection(activity, DEFAULT_ROS_MASTER_URI);
        rosConnection.initialize();

        // ✅ 新增：初始化点云发布器
        localPointCloudPublisher = new LocalPointCloudPublisher(activity, this);

        // 添加状态视图
        addRosStatusView();

        // 定期更新状态
        startStatusUpdates();
    }

    // ===== 点云发布相关方法 =====

    /**
     * ✅ 修改：使用独立的点云发布器
     */
    public void publishLocalPointCloud(float[] localPointCloud) {
        if (localPointCloudPublisher != null) {
            localPointCloudPublisher.publishLocalPointCloud(localPointCloud);
        }
    }

    /**
     * ✅ 修改：使用独立的点云发布器进行测试
     */
    public void testPublishLocalPointCloud() {
        if (localPointCloudPublisher != null) {
            localPointCloudPublisher.testPublishLocalPointCloud();
        } else {
            Log.w(TAG, "点云发布器未初始化");
        }
    }

    /**
     * ✅ 新增：检查点云发布器是否就绪
     */
    public boolean isLocalPointCloudPublisherReady() {
        return localPointCloudPublisher != null && localPointCloudPublisher.isReady();
    }

    /**
     * ✅ 新增：获取点云发布器统计信息
     */
    public String getLocalPointCloudStatistics() {
        return localPointCloudPublisher != null ?
                localPointCloudPublisher.getStatistics() : "点云发布器未初始化";
    }

    /**
     * ✅ 新增：检查点云发布器是否有订阅者
     */
    public boolean hasLocalPointCloudSubscribers() {
        return localPointCloudPublisher != null && localPointCloudPublisher.hasSubscribers();
    }

    /**
     * ✅ 新增：重置点云发布器统计信息
     */
    public void resetLocalPointCloudStatistics() {
        if (localPointCloudPublisher != null) {
            localPointCloudPublisher.resetStatistics();
        }
    }

    // ===== 深度图发布相关方法 =====

    /**
     * 检查深度图发布器是否有订阅者
     */
    public boolean hasDepthImageSubscribers() {
        if (rosConnection == null || !rosConnection.isConnected()) {
            return false;
        }
        try {
            return rosConnection.hasDepthImageSubscribers();
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * 发布深度图到ROS
     * @param depthMatAddr 深度图Mat对象地址
     */
    public void publishDepthImage(long depthMatAddr) {
        if (rosConnection != null && rosConnection.isConnected()) {
            rosConnection.publishDepthImage(depthMatAddr);
        }
    }

    // ===== 地图发布相关方法 =====

    /**
     * 发布静态地图到/map话题（使用静态地图发布器）
     * @param mapData 地图占用栅格数据（一维数组）
     * @param mapInfo 地图信息
     */
    public void publishMap(byte[] mapData, MapServerManager.MapInfo mapInfo) {
        if (rosConnection != null && rosConnection.isConnected()) {
            try {
                // 转换为网格地图格式
                byte[][] gridData = convertToGridFormat(mapData, mapInfo.width, mapInfo.height);

                // 创建GridMapInfo对象
                GridMapBuilder.GridMapInfo gridMapInfo = new GridMapBuilder.GridMapInfo(
                        mapInfo.width,
                        mapInfo.height,
                        (float)mapInfo.resolution,
                        (float)mapInfo.originX,
                        (float)mapInfo.originY
                );

                // 设置时间戳
                gridMapInfo.timestamp = mapInfo.mapLoadTime;

                // 使用静态地图发布功能
                if (rosConnection.isStaticMapPublisherReady()) {
                    rosConnection.publishStaticMap(gridData, gridMapInfo);
                    Log.i(TAG, "静态地图发布成功到/map话题");
                } else {
                    Log.w(TAG, "静态地图发布器未就绪");
                }

            } catch (Exception e) {
                Log.e(TAG, "发布静态地图时出错: " + e.getMessage(), e);
            }
        } else {
            Log.w(TAG, "ROS未连接，无法发布静态地图");
        }
    }

    /**
     * 将一维地图数据转换为二维网格格式
     * @param mapData 一维地图数据
     * @param width 地图宽度
     * @param height 地图高度
     * @return 二维网格数据
     */
    private byte[][] convertToGridFormat(byte[] mapData, int width, int height) {
        byte[][] gridData = new byte[height][width];

        // 转换数据格式：从一维数组转换为二维数组
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int index = y * width + x;
                if (index < mapData.length) {
                    gridData[y][x] = mapData[index];
                } else {
                    gridData[y][x] = -1; // 未知区域
                }
            }
        }

        return gridData;
    }

    /**
     * 发布网格地图到 /grid_map 话题（用于实时SLAM地图）
     * @param mapData 地图数据矩阵
     * @param mapInfo 地图信息
     */
    public void publishGridMap(byte[][] mapData, GridMapBuilder.GridMapInfo mapInfo) {
        if (rosConnection != null && rosConnection.isConnected() &&
                rosConnection.isGridMapPublisherReady()) {
            rosConnection.publishGridMap(mapData, mapInfo);
        }
    }

    // ===== 地图发布器状态检查方法 =====

    /**
     * 检查静态地图发布器是否就绪
     * @return 发布器状态
     */
    public boolean isMapPublisherReady() {
        return rosConnection != null && rosConnection.isStaticMapPublisherReady();
    }

    /**
     * 检查网格地图发布器是否就绪
     * @return 发布器状态
     */
    public boolean isGridMapPublisherReady() {
        return rosConnection != null && rosConnection.isGridMapPublisherReady();
    }

    /**
     * 检查静态地图发布器是否就绪
     * @return 发布器状态
     */
    public boolean isStaticMapPublisherReady() {
        return rosConnection != null && rosConnection.isStaticMapPublisherReady();
    }

    // ===== 图像发布相关方法 =====

    /**
     * 发布地图数量
     */
    public void publishMapCount(int mapCount) {
        if (rosConnection != null && rosConnection.isConnected()) {
            rosConnection.publishMapCount(mapCount);
        }
    }

    /**
     * 发布双目图像到ROS
     * @param leftImage 左相机图像
     * @param rightImage 右相机图像
     */
    public void publishStereoImages(Mat leftImage, Mat rightImage) {
        if (rosConnection != null && rosConnection.isConnected()) {
            rosConnection.publishStereoImages(leftImage, rightImage);
        }
    }

    // ===== SLAM数据发布相关方法 =====

    /**
     * 发布关键帧建图数据到 /pts_and_pose 话题
     * @param keyFrameData 关键帧数据数组
     * 格式: [kf_pose(7个float), map_points(3n个float)]
     * 其中 kf_pose = [x, y, z, qx, qy, qz, qw]（已转换坐标系）
     *      map_points = [x1, y1, z1, x2, y2, z2, ...]（已转换坐标系）
     */
    public void publishKeyFrameData(float[] keyFrameData) {
        if (rosConnection != null && rosConnection.isConnected() &&
                rosConnection.isPtsAndPosePublisherReady()) {
            rosConnection.publishKeyFrameData(keyFrameData);
        }
    }

    /**
     * 发布当前跟踪到的地图点（实时数据，非关键帧）
     * @param mapPoints 地图点数组 [x1, y1, z1, x2, y2, z2, ...]
     * @param cameraPose 相机位姿数组 [x, y, z, qx, qy, qz, qw]
     * @deprecated 此方法用于实时跟踪数据，不再发布到 /pts_and_pose
     */
    @Deprecated
    public void publishTrackedMapPoints(float[] mapPoints, float[] cameraPose) {
        // 不再发布到 /pts_and_pose，可以发布到其他话题用于调试
        // if (rosConnection != null && rosConnection.isConnected()) {
        //     rosConnection.publishTrackedMapPoints(mapPoints, cameraPose);  // 已移除
        // }
    }

    /**
     * 发布所有关键帧及其地图点
     * @param keyframeData 关键帧和地图点数据
     * 格式: [关键帧数量, kf1.x, kf1.y, kf1.z, kf1.qx, kf1.qy, kf1.qz, kf1.qw,
     *        kf1点数量, kf1_pt1.x, kf1_pt1.y, kf1_pt1.z, ...]
     */
    public void publishAllKeyframeData(float[] keyframeData) {
        if (rosConnection != null && rosConnection.isConnected() &&
                rosConnection.isAllKfAndPtsPublisherReady()) {
            rosConnection.publishAllKeyframeData(keyframeData);
        }
    }

    /**
     * 发布当前相机位姿到 /orb_slam3/camera_pose 话题
     * @param cameraPose 相机位姿数组 [x, y, z, qx, qy, qz, qw]
     */
    public void publishCurrentCameraPose(float[] cameraPose) {
        if (rosConnection != null && rosConnection.isConnected() &&
                rosConnection.isCurCameraPosePublisherReady()) {
            rosConnection.publishCurrentCameraPose(cameraPose);
        }
    }

    // ===== 发布器状态检查方法 =====

    /**
     * 检查ROS是否已连接
     * @return 连接状态
     */
    public boolean isConnected() {
        return rosConnection != null && rosConnection.isConnected();
    }

    /**
     * 检查 /pts_and_pose 发布器是否就绪
     * @return 发布器状态
     */
    public boolean isPtsAndPosePublisherReady() {
        return rosConnection != null && rosConnection.isPtsAndPosePublisherReady();
    }

    /**
     * 检查 /all_kf_and_pts 发布器是否就绪
     * @return 发布器状态
     */
    public boolean isAllKfAndPtsPublisherReady() {
        return rosConnection != null && rosConnection.isAllKfAndPtsPublisherReady();
    }

    /**
     * 检查相机位姿发布器是否就绪 (/orb_slam3/camera_pose - PoseStamped)
     * @return 发布器状态
     */
    public boolean isCurCameraPosePublisherReady() {
        return rosConnection != null && rosConnection.isCurCameraPosePublisherReady();
    }

    // ===== UI状态显示相关方法 =====

    /**
     * 添加ROS状态显示视图
     */
    private void addRosStatusView() {
        // 创建一个状态指示器背景
        statusIndicator.setShape(GradientDrawable.OVAL);
        statusIndicator.setSize(20, 20);
        statusIndicator.setColor(Color.GRAY);

        // 创建状态文本视图
        rosStatusText = new TextView(activity);
        rosStatusText.setTextColor(Color.WHITE);
        rosStatusText.setBackgroundColor(Color.argb(128, 0, 0, 0));
        rosStatusText.setPadding(40, 10, 20, 10);
        rosStatusText.setCompoundDrawablePadding(10);
        rosStatusText.setTextSize(TypedValue.COMPLEX_UNIT_SP, 12);
        //rosStatusText.setText("ROS: 正在连接...");
        //rosStatusText.setCompoundDrawablesWithIntrinsicBounds(statusIndicator, null, null, null);

        // 添加到布局
        ViewGroup rootView = activity.findViewById(android.R.id.content);
        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(
                FrameLayout.LayoutParams.WRAP_CONTENT, FrameLayout.LayoutParams.WRAP_CONTENT);
        params.gravity = Gravity.TOP | Gravity.START;
        params.setMargins(20, 20, 0, 0);
        rootView.addView(rosStatusText, params);
    }

    /**
     * 启动定期状态更新
     */
    private void startStatusUpdates() {
        statusUpdateHandler = new Handler();
        statusUpdateHandler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if (rosConnection != null) {
                    boolean isConnected = rosConnection.isConnected();
                    statusIndicator.setColor(isConnected ? Color.GREEN : Color.RED);

                    // 更新状态文本，包含点云发布器状态
                    String statusText = "ROS: " + (isConnected ? "已连接" : "未连接");
                    if (isConnected && localPointCloudPublisher != null) {
                        boolean pointCloudReady = localPointCloudPublisher.isReady();
                    }

                    //rosStatusText.setText(statusText);
                    //rosStatusText.setCompoundDrawablesWithIntrinsicBounds(statusIndicator, null, null, null);
                }
                statusUpdateHandler.postDelayed(this, 1000);
            }
        }, 1000);
    }

    // ===== 系统状态和调试方法 =====

    /**
     * 获取ROS系统状态信息
     */
    public String getSystemStatus() {
        StringBuilder status = new StringBuilder();
        status.append("=== ROS系统状态 ===\n");
        status.append("ROS连接: ").append(isConnected() ? "✅" : "❌").append("\n");
        status.append("静态地图发布器: ").append(isStaticMapPublisherReady() ? "✅" : "❌").append("\n");
        status.append("网格地图发布器: ").append(isGridMapPublisherReady() ? "✅" : "❌").append("\n");
        status.append("相机位姿发布器: ").append(isCurCameraPosePublisherReady() ? "✅" : "❌").append("\n");
        status.append("关键帧发布器: ").append(isPtsAndPosePublisherReady() ? "✅" : "❌").append("\n");
        status.append("完整关键帧发布器: ").append(isAllKfAndPtsPublisherReady() ? "✅" : "❌").append("\n");
        status.append("点云发布器: ").append(isLocalPointCloudPublisherReady() ? "✅" : "❌").append("\n");

        if (localPointCloudPublisher != null) {
            status.append("\n=== 点云发布器统计 ===\n");
            status.append(localPointCloudPublisher.getStatistics());
        }

        return status.toString();
    }

    /**
     * 重置所有统计信息
     */
    public void resetAllStatistics() {
        if (localPointCloudPublisher != null) {
            localPointCloudPublisher.resetStatistics();
        }
        Log.i(TAG, "所有统计信息已重置");
    }

    /**
     * 测试所有发布器
     */
    public void testAllPublishers() {
        Log.i(TAG, "开始测试所有发布器...");

        // 测试点云发布器
        if (localPointCloudPublisher != null && localPointCloudPublisher.isReady()) {
            localPointCloudPublisher.testPublishLocalPointCloud();
            Log.i(TAG, "点云发布器测试完成");
        }

        // 测试地图数量发布
        if (isConnected()) {
            publishMapCount(9999); // 测试数据
            Log.i(TAG, "地图数量发布器测试完成");
        }

        Log.i(TAG, "所有发布器测试完成");
    }

    // ===== 资源管理方法 =====

    /**
     * 关闭ROS功能并释放资源
     */
    public void shutdown() {
        Log.i(TAG, "正在关闭ROS管理器...");

        if (statusUpdateHandler != null) {
            statusUpdateHandler.removeCallbacksAndMessages(null);
            statusUpdateHandler = null;
        }

        // 关闭点云发布器
        if (localPointCloudPublisher != null) {
            localPointCloudPublisher.shutdown();
            localPointCloudPublisher = null;
        }

        if (rosConnection != null) {
            rosConnection.shutdown();
            rosConnection = null;
        }

        if (rosStatusText != null && rosStatusText.getParent() != null) {
            ((ViewGroup) rosStatusText.getParent()).removeView(rosStatusText);
            rosStatusText = null;
        }

        Log.i(TAG, "ROS管理器关闭完成");
    }

    /**
     * 重新初始化ROS连接
     */
    public void reinitialize() {
        Log.i(TAG, "重新初始化ROS管理器...");

        // 关闭现有连接
        if (rosConnection != null) {
            rosConnection.shutdown();
        }

        // 重新初始化
        try {
            Thread.sleep(2000); // 等待2秒
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        initialize();
    }

    /**
     * 获取ROS连接对象（用于高级操作）
     * @return ROS连接对象
     */
    public RosConnection getRosConnection() {
        return rosConnection;
    }

    /**
     * 获取点云发布器对象（用于高级操作）
     * @return 点云发布器对象
     */
    public LocalPointCloudPublisher getLocalPointCloudPublisher() {
        return localPointCloudPublisher;
    }
}