// LocalPointCloudPublisher.java
package com.vslam.orbslam3.vslamactivity;

import android.content.Context;
import android.util.Log;
import android.widget.Toast;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import sensor_msgs.PointCloud2;
import sensor_msgs.PointField;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;

/**
 * 局部点云发布器类 - 独立处理 /orb_slam3/local_pointcloud 话题发布
 */
public class LocalPointCloudPublisher {
    private static final String TAG = "LocalPointCloudPublisher";
    private static final String TOPIC_NAME = "/orb_slam3/local_pointcloud";
    private static final boolean DEBUG = true;

    private final Context context;
    private final VslamActivity activity;
    private RosManager rosManager;

    // ROS发布器
    private volatile Publisher<sensor_msgs.PointCloud2> localPointCloudPublisher;
    private ConnectedNode connectedNode;

    // 状态变量
    private AtomicBoolean isInitialized = new AtomicBoolean(false);
    private AtomicBoolean isPublishing = new AtomicBoolean(false);

    // 统计信息
    private long totalPointsPublished = 0;
    private long publishCount = 0;
    private long lastPublishTime = 0;

    // 调试信息管理器
    private DebugLogger debugLogger;

    /**
     * 构造函数
     * @param activity VSLAM活动实例
     * @param rosManager ROS管理器实例
     */
    public LocalPointCloudPublisher(VslamActivity activity, RosManager rosManager) {
        this.activity = activity;
        this.context = activity;
        this.rosManager = rosManager;

        initializeRosPublisher();
    }

    /**
     * 初始化ROS发布器
     */
    private void initializeRosPublisher() {
        new Thread(() -> {
            int retryCount = 0;
            while (!isInitialized.get() && retryCount < 30) { // 最多等待30秒
                try {
                    Thread.sleep(1000);
                    if (rosManager != null && rosManager.isConnected()) {
                        connectedNode = getConnectedNodeFromRosManager();
                        if (connectedNode != null) {
                            createPointCloudPublisher();
                            break;
                        }
                    }
                    retryCount++;
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }

            if (!isInitialized.get()) {
                Log.w(TAG, "点云发布器初始化超时");
                //showToast("点云发布器初始化失败");
            }
        }).start();
    }

    /**
     * 从RosManager获取ConnectedNode
     */
    private ConnectedNode getConnectedNodeFromRosManager() {
        try {
            // 通过反射获取RosConnection中的ConnectedNode
            java.lang.reflect.Field rosConnectionField = rosManager.getClass().getDeclaredField("rosConnection");
            rosConnectionField.setAccessible(true);
            Object rosConnection = rosConnectionField.get(rosManager);

            if (rosConnection != null) {
                java.lang.reflect.Field connectedNodeField = rosConnection.getClass().getDeclaredField("connectedNode");
                connectedNodeField.setAccessible(true);
                return (ConnectedNode) connectedNodeField.get(rosConnection);
            }
        } catch (Exception e) {
            Log.e(TAG, "获取ConnectedNode失败: " + e.getMessage(), e);
        }
        return null;
    }

    /**
     * 创建点云发布器
     */
    private void createPointCloudPublisher() {
        try {
            if (connectedNode != null) {
                localPointCloudPublisher = connectedNode.newPublisher(
                        TOPIC_NAME,
                        sensor_msgs.PointCloud2._TYPE
                );

                // 初始化调试日志器
                debugLogger = new DebugLogger(context);

                isInitialized.set(true);

                Log.i(TAG, "点云发布器初始化成功，话题: " + TOPIC_NAME);
                activity.runOnUiThread(() -> {
                    //showToast("点云发布器已就绪");
                });


            }
        } catch (Exception e) {
            Log.e(TAG, "创建点云发布器失败: " + e.getMessage(), e);
            //showToast("创建点云发布器失败");

            if (debugLogger != null) {
                debugLogger.updateDebugTextView("❌ 点云发布器初始化失败: " + e.getMessage());
            }
        }
    }

    /**
     * 发布局部点云数据
     * @param localPointCloud 点云数据数组，格式：[x1, y1, z1, r1, g1, b1, x2, y2, z2, r2, g2, b2, ...]
     */
    public void publishLocalPointCloud(float[] localPointCloud) {
        // 检查连接和数据有效性
        if (!isReady() || localPointCloud == null || localPointCloud.length == 0) {
            if (DEBUG) {
                Log.d(TAG, "无法发布局部点云: " +
                        "就绪状态=" + isReady() +
                        ", 数据长度=" + (localPointCloud == null ? "null" : localPointCloud.length));
            }
            return;
        }

        if (localPointCloud.length % 6 != 0) {
            Log.e(TAG, "点云数据长度必须是6的倍数，当前长度: " + localPointCloud.length);
            if (debugLogger != null) {
                debugLogger.updateDebugTextView("❌ 点云数据格式错误，长度必须是6的倍数");
            }
            return;
        }

        // 防止重复发布
        if (isPublishing.get()) {
            if (DEBUG) Log.d(TAG, "正在发布点云，跳过本次发布");
            return;
        }

        isPublishing.set(true);

        try {
            // 创建 PointCloud2 消息
            PointCloud2 pointCloudMsg = localPointCloudPublisher.newMessage();

            // 设置消息头
            pointCloudMsg.getHeader().setFrameId("map");
            pointCloudMsg.getHeader().setStamp(connectedNode.getCurrentTime());
            pointCloudMsg.setIsBigendian(false);

            // 设置点云字段：x, y, z, r, g, b
            List<PointField> fields = createPointFields();
            pointCloudMsg.setFields(fields);

            // 计算点数和设置消息属性
            int numPoints = localPointCloud.length / 6;
            pointCloudMsg.setWidth(numPoints);
            pointCloudMsg.setHeight(1);
            pointCloudMsg.setPointStep(24); // 6 fields * 4 bytes each
            pointCloudMsg.setRowStep(numPoints * 24);

            // 创建并填充数据
            byte[] dataArray = createPointCloudData(localPointCloud, numPoints);

            // 设置数据到消息
            ChannelBuffer channelBuffer = ChannelBuffers.wrappedBuffer(dataArray);
            pointCloudMsg.setData(channelBuffer);

            // 发布消息
            localPointCloudPublisher.publish(pointCloudMsg);

            // 更新统计信息
            updateStatistics(numPoints);

            if (DEBUG) {
                Log.d(TAG, String.format("✅ 发布点云成功: %d个点, 话题: %s", numPoints, TOPIC_NAME));
            }

            if (debugLogger != null) {
                debugLogger.updateDebugTextView(String.format("📊 发布点云: %d个点, 总计: %d次",
                        numPoints, publishCount));
            }

        } catch (Exception e) {
            //Log.e(TAG, "发布点云时出错: " + e.getMessage(), e);
            if (debugLogger != null) {
                //debugLogger.updateDebugTextView("❌ 发布点云失败: " + e.getMessage());
            }
        } finally {
            isPublishing.set(false);
        }
    }

    /**
     * 创建点云字段定义
     */
    private List<PointField> createPointFields() {
        List<PointField> fields = new ArrayList<>();

        String[] fieldNames = {"x", "y", "z", "r", "g", "b"};
        int[] offsets = {0, 4, 8, 12, 16, 20};
        byte[] datatypes = {
                (byte) PointField.FLOAT32,  // x
                (byte) PointField.FLOAT32,  // y
                (byte) PointField.FLOAT32,  // z
                (byte) PointField.UINT8,    // r
                (byte) PointField.UINT8,    // g
                (byte) PointField.UINT8     // b
        };

        for (int i = 0; i < fieldNames.length; i++) {
            PointField field = connectedNode.getTopicMessageFactory().newFromType(PointField._TYPE);
            field.setName(fieldNames[i]);
            field.setOffset(offsets[i]);
            field.setDatatype(datatypes[i]);
            field.setCount(1);
            fields.add(field);
        }

        return fields;
    }

    /**
     * 创建点云数据字节数组
     */
    private byte[] createPointCloudData(float[] localPointCloud, int numPoints) {
        byte[] dataArray = new byte[numPoints * 24]; // 24 bytes per point
        ByteBuffer buffer = ByteBuffer.wrap(dataArray);

        for (int i = 0; i < localPointCloud.length; i += 6) {
            // 3D坐标 (x, y, z)
            float x = localPointCloud[i];
            float y = localPointCloud[i + 1];
            float z = localPointCloud[i + 2];

            // 颜色 (r, g, b) - 转换为0-255范围
            int r = Math.max(0, Math.min((int) (localPointCloud[i + 3] * 255), 255));
            int g = Math.max(0, Math.min((int) (localPointCloud[i + 4] * 255), 255));
            int b = Math.max(0, Math.min((int) (localPointCloud[i + 5] * 255), 255));

            // 写入缓冲区
            buffer.putFloat(x);
            buffer.putFloat(y);
            buffer.putFloat(z);
            buffer.put((byte) r);
            buffer.put((byte) g);
            buffer.put((byte) b);
            buffer.put((byte) 0); // 填充字节，保持4字节对齐
        }

        return dataArray;
    }

    /**
     * 更新统计信息
     */
    private void updateStatistics(int numPoints) {
        publishCount++;
        totalPointsPublished += numPoints;
        lastPublishTime = System.currentTimeMillis();
    }

    /**
     * 测试发布硬编码的点云数据
     */
    public void testPublishLocalPointCloud() {
        if (!isReady()) {
            Log.w(TAG, "点云发布器未就绪，无法测试");
            //showToast("点云发布器未就绪");
            return;
        }

        // 硬编码的测试点云数据
        float[] testPointCloud = {
                // 第一个点：红色
                1.0f, 2.0f, 3.0f,    // x, y, z
                1.0f, 0.0f, 0.0f,    // r, g, b (红色)

                // 第二个点：绿色
                2.0f, 3.0f, 4.0f,    // x, y, z
                0.0f, 1.0f, 0.0f,    // r, g, b (绿色)

                // 第三个点：蓝色
                3.0f, 4.0f, 5.0f,    // x, y, z
                0.0f, 0.0f, 1.0f,    // r, g, b (蓝色)
        };

        publishLocalPointCloud(testPointCloud);

        Log.i(TAG, "测试点云发布完成");
        //showToast("测试点云已发布");
    }

    /**
     * 检查发布器是否就绪
     */
    public boolean isReady() {
        return isInitialized.get() &&
                localPointCloudPublisher != null &&
                connectedNode != null &&
                rosManager != null &&
                rosManager.isConnected();
    }

    /**
     * 获取发布器订阅者数量
     */
    public int getSubscriberCount() {
        if (localPointCloudPublisher == null) {
            return 0;
        }

        try {
            return localPointCloudPublisher.getNumberOfSubscribers();
        } catch (Exception e) {
            Log.e(TAG, "获取订阅者数量失败: " + e.getMessage(), e);
            return 0;
        }
    }

    /**
     * 检查是否有订阅者
     */
    public boolean hasSubscribers() {
        return getSubscriberCount() > 0;
    }

    /**
     * 获取统计信息
     */
    public String getStatistics() {
        return String.format(
                "点云发布统计:\n" +
                        "发布次数: %d\n" +
                        "总点数: %d\n" +
                        "订阅者: %d\n" +
                        "最后发布: %s\n" +
                        "状态: %s",
                publishCount,
                totalPointsPublished,
                getSubscriberCount(),
                lastPublishTime > 0 ? new java.util.Date(lastPublishTime).toString() : "从未发布",
                isReady() ? "就绪" : "未就绪"
        );
    }

    /**
     * 重置统计信息
     */
    public void resetStatistics() {
        publishCount = 0;
        totalPointsPublished = 0;
        lastPublishTime = 0;

        if (debugLogger != null) {
            debugLogger.updateDebugTextView("📊 点云发布统计信息已重置");
        }
    }

    /**
     * 显示Toast消息
     */
    private void showToast(String message) {
        if (context != null) {
            activity.runOnUiThread(() ->
                    Toast.makeText(context, message, Toast.LENGTH_SHORT).show());
        }
    }

    /**
     * 手动重试初始化
     */
    public void retryInitialization() {
        if (!isInitialized.get()) {
            Log.i(TAG, "手动重试初始化点云发布器");
            initializeRosPublisher();
        }
    }

    /**
     * 清理资源
     */
    public void shutdown() {
        Log.i(TAG, "正在关闭点云发布器...");

        isInitialized.set(false);
        isPublishing.set(false);

        if (localPointCloudPublisher != null) {
            try {
                // ROS发布器会在节点关闭时自动清理
                localPointCloudPublisher = null;
                Log.i(TAG, "点云发布器已清理");
            } catch (Exception e) {
                Log.e(TAG, "清理点云发布器时出错: " + e.getMessage(), e);
            }
        }

        rosManager = null;
        connectedNode = null;
        debugLogger = null;

        Log.i(TAG, "点云发布器关闭完成");
    }
}