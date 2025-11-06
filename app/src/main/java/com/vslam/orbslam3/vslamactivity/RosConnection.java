// RosConnection.java
package com.vslam.orbslam3.vslamactivity;

import android.content.Context;
import android.util.Log;
import android.widget.Toast;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import sensor_msgs.PointCloud2;
import sensor_msgs.PointField;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.io.ByteArrayOutputStream;
import java.net.InetAddress;
import java.net.Inet4Address;
import java.net.NetworkInterface;
import java.net.URI;
import java.util.Enumeration;
import java.util.concurrent.atomic.AtomicBoolean;

import sensor_msgs.CompressedImage;
import geometry_msgs.PoseStamped;
import geometry_msgs.PoseArray;
import geometry_msgs.Pose;
import nav_msgs.OccupancyGrid;
import sensor_msgs.PointCloud2;
import std_msgs.Int32;

import java.nio.ByteOrder;
import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.opencv.imgcodecs.Imgcodecs;

/**
 * ROS连接管理类，用于处理ROS连接和消息发布
 */
public class RosConnection implements NodeMain {

    private static final String TAG = "RosConnection";
    private static final boolean DEBUG = true;

    // 添加网格地图专用的缓冲流
    private final ChannelBufferOutputStream gridMapChannelBufferStream =
            new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());

    // ROS相关变量
    private volatile Publisher<CompressedImage> leftCompressedImagePublisher;
    private volatile Publisher<CompressedImage> rightCompressedImagePublisher;
    private volatile Publisher<std_msgs.Int32> mapsCountPublisher;
    private volatile Publisher<sensor_msgs.Image> depthImagePublisher;

    // 用于处理ROS消息缓冲区的输出流
    private final ChannelBufferOutputStream leftChannelBufferStream =
            new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
    private final ChannelBufferOutputStream rightChannelBufferStream =
            new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());

    private AtomicBoolean isConnected = new AtomicBoolean(false);
    private NodeMainExecutor nodeMainExecutor;
    private NodeConfiguration nodeConfiguration;
    private volatile ConnectedNode connectedNode;

    // 错误计数器
    private int errorCount = 0;
    private static final int MAX_ERROR_COUNT = 5;
    private long lastErrorTime = 0;

    // SLAM数据发布器
    private volatile Publisher<geometry_msgs.PoseArray> mapPointsPublisher;
    private volatile Publisher<geometry_msgs.PoseArray> keyframesPublisher;
    private volatile Publisher<geometry_msgs.PoseStamped> cameraPosePublisher;
    private volatile Publisher<nav_msgs.OccupancyGrid> gridMapPublisher;

    // 静态地图发布器
    private volatile Publisher<nav_msgs.OccupancyGrid> staticMapPublisher;

    private DebugLogger debugLogger;
    private final Context context;
    private String masterUri;

    // 构造函数
    public RosConnection(Context context, String masterUri) {
        this.context = context;
        this.masterUri = masterUri;
    }

    /**
     * 初始化ROS连接
     */
    public void initialize() {
        try {
            Thread thread = new Thread(() -> {
                try {
                    URI masterUriObj = URI.create(masterUri);
                    InetAddress localAddress = getLocalIPAddress();

                    if (localAddress == null) {
                        //showToast("无法获取本地IP地址");
                        return;
                    }

                    Log.i(TAG, "使用本地IP地址: " + localAddress.getHostAddress());

                    nodeConfiguration = NodeConfiguration.newPublic(localAddress.getHostAddress());
                    nodeConfiguration.setMasterUri(masterUriObj);

                    // 使用当前对象作为NodeMain
                    nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
                    nodeMainExecutor.execute(this, nodeConfiguration);

                    //showToast("ROS连接已初始化");
                    Log.i(TAG, "ROS连接已初始化，主节点URI: " + masterUri);

                } catch (Exception e) {
                    final String errorMsg = e.getMessage();
                    //showToast("ROS连接失败: " + errorMsg);
                    Log.e(TAG, "ROS连接失败: " + errorMsg, e);
                }
            });
            thread.start();

        } catch (Exception e) {
            //showToast("无法启动ROS连接: " + e.getMessage());
            Log.e(TAG, "无法启动ROS连接", e);
        }
    }

    /**
     * 获取连接状态
     */
    public boolean isConnected() {
        return isConnected.get() && connectedNode != null;
    }

    /**
     * 检查左相机图像发布器是否准备就绪
     */
    public boolean isLeftPublisherReady() {
        return leftCompressedImagePublisher != null;
    }

    /**
     * 检查右相机图像发布器是否准备就绪
     */
    public boolean isRightPublisherReady() {
        return rightCompressedImagePublisher != null;
    }

    /**
     * 发布地图数量
     */
    public void publishMapCount(int mapCount) {
        if (!isConnected() || mapsCountPublisher == null) {
            return;
        }

        try {
            Int32 msg = mapsCountPublisher.newMessage();
            msg.setData(mapCount);
            mapsCountPublisher.publish(msg);
            if (DEBUG) Log.d(TAG, "Published map count: " + mapCount);
        } catch (Exception e) {
            Log.e(TAG, "Error publishing map count: " + e.getMessage(), e);
        }
    }

    /**
     * 发布立体相机图像
     */
    public void publishStereoImages(Mat leftImage, Mat rightImage) {
        // 检查连接状态
        if (!isConnected()) {
            if (DEBUG) Log.d(TAG, "ROS未连接，无法发布图像");
            return;
        }

        // 检查发布器是否就绪
        if (leftCompressedImagePublisher == null || rightCompressedImagePublisher == null) {
            Log.e(TAG, "ROS发布器未就绪，左: " + (leftCompressedImagePublisher != null) +
                    ", 右: " + (rightCompressedImagePublisher != null));
            return;
        }

        // 检查图像状态
        if (leftImage == null || leftImage.empty()) {
            Log.e(TAG, "左图像为空，无法发布");
            return;
        }

        if (rightImage == null || rightImage.empty()) {
            Log.e(TAG, "右图像为空，无法发布");
            return;
        }

        try {
            // 创建和发布左相机图像
            CompressedImage leftMsg = leftCompressedImagePublisher.newMessage();
            if (publishMatAsJpeg(leftImage, leftMsg, leftChannelBufferStream, "left_camera")) {
                leftCompressedImagePublisher.publish(leftMsg);
                if (DEBUG) Log.d(TAG, "左相机图像已发布");
            }

            // 创建和发布右相机图像
            CompressedImage rightMsg = rightCompressedImagePublisher.newMessage();
            if (publishMatAsJpeg(rightImage, rightMsg, rightChannelBufferStream, "right_camera")) {
                rightCompressedImagePublisher.publish(rightMsg);
                if (DEBUG) Log.d(TAG, "右相机图像已发布");
            }

            // 成功发布后重置错误计数
            errorCount = 0;

        } catch (Exception e) {
            handlePublishError(e);
        }
    }

    /**
     * 将Mat图像转换为JPEG并添加到压缩图像消息
     */
    private boolean publishMatAsJpeg(Mat image, CompressedImage msg,
                                     ChannelBufferOutputStream stream, String frameId) {
        if (image == null || image.empty() || msg == null || stream == null) {
            return false;
        }

        Mat processedImage = null;
        MatOfByte matOfByte = new MatOfByte();
        boolean success = false;

        try {
            // 转换图像格式
            if (image.channels() == 3) {
                processedImage = new Mat();
                Imgproc.cvtColor(image, processedImage, Imgproc.COLOR_RGB2BGR);
            } else {
                processedImage = image;
            }

            // 编码为JPEG
            boolean encodeSuccess = Imgcodecs.imencode(".jpg", processedImage, matOfByte);
            if (!encodeSuccess) {
                Log.e(TAG, "图像编码失败");
                return false;
            }

            byte[] imageData = matOfByte.toArray();
            if (imageData == null || imageData.length == 0) {
                Log.e(TAG, "编码后的图像数据为空");
                return false;
            }

            // 设置消息属性
            msg.setFormat("jpeg");
            msg.getHeader().setStamp(connectedNode.getCurrentTime());
            msg.getHeader().setFrameId(frameId);

            // 使用ChannelBufferOutputStream写入数据
            synchronized (stream) {
                stream.buffer().clear();
                stream.write(imageData);
                msg.setData(stream.buffer().copy());
            }

            success = true;
        } catch (Exception e) {
            Log.e(TAG, "处理图像时出错: " + e.getMessage(), e);
        } finally {
            // 释放资源
            if (processedImage != null && processedImage != image) {
                processedImage.release();
            }
            matOfByte.release();
        }

        return success;
    }

    // ===== SLAM数据发布方法 =====

    /**
     * 发布关键帧建图数据到 /pts_and_pose 话题
     */
    public void publishKeyFrameData(float[] keyFrameData) {
        if (!isConnected() || mapPointsPublisher == null || keyFrameData == null || keyFrameData.length < 7) {
            return;
        }

        try {
            // 创建PoseArray消息
            geometry_msgs.PoseArray poseArray = mapPointsPublisher.newMessage();
            poseArray.getHeader().setFrameId("map");
            poseArray.getHeader().setStamp(connectedNode.getCurrentTime());

            // 第一个pose：关键帧位姿
            geometry_msgs.Pose kfPose = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
            kfPose.getPosition().setX(keyFrameData[0]);
            kfPose.getPosition().setY(keyFrameData[1]);
            kfPose.getPosition().setZ(keyFrameData[2]);
            kfPose.getOrientation().setX(keyFrameData[3]);
            kfPose.getOrientation().setY(keyFrameData[4]);
            kfPose.getOrientation().setZ(keyFrameData[5]);
            kfPose.getOrientation().setW(keyFrameData[6]);
            poseArray.getPoses().add(kfPose);

            // 后续poses：地图点
            int numPoints = (keyFrameData.length - 7) / 3;
            for (int i = 0; i < numPoints; i++) {
                geometry_msgs.Pose pointPose = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);

                int baseIndex = 7 + i * 3;
                pointPose.getPosition().setX(keyFrameData[baseIndex]);
                pointPose.getPosition().setY(keyFrameData[baseIndex + 1]);
                pointPose.getPosition().setZ(keyFrameData[baseIndex + 2]);

                pointPose.getOrientation().setX(0);
                pointPose.getOrientation().setY(0);
                pointPose.getOrientation().setZ(0);
                pointPose.getOrientation().setW(1);

                poseArray.getPoses().add(pointPose);
            }

            // 发布消息
            mapPointsPublisher.publish(poseArray);
            if (DEBUG) Log.d(TAG, "Published keyframe data with " + numPoints + " map points");

        } catch (Exception e) {
            Log.e(TAG, "Error publishing keyframe data: " + e.getMessage(), e);
        }
    }

    /**
     * 发布所有关键帧数据，使用PoseArray
     */
    public void publishAllKeyframeData(float[] data) {
        if (!isConnected() || keyframesPublisher == null || data == null || data.length < 1) {
            return;
        }

        try {
            // 创建PoseArray消息
            geometry_msgs.PoseArray poseArray = keyframesPublisher.newMessage();
            poseArray.getHeader().setFrameId("map");
            poseArray.getHeader().setStamp(connectedNode.getCurrentTime());

            // 提取关键帧数量
            int kfCount = (int)data[0];
            int index = 1;

            // 第一个pose：关键帧总数
            geometry_msgs.Pose kfCountPose = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
            kfCountPose.getPosition().setX(kfCount);
            kfCountPose.getPosition().setY(kfCount);
            kfCountPose.getPosition().setZ(kfCount);
            kfCountPose.getOrientation().setW(1.0);
            poseArray.getPoses().add(kfCountPose);

            // 处理每个关键帧
            for (int i = 0; i < kfCount; i++) {
                // 添加关键帧位姿
                geometry_msgs.Pose kfPose = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
                kfPose.getPosition().setX(data[index++]);
                kfPose.getPosition().setY(data[index++]);
                kfPose.getPosition().setZ(data[index++]);
                kfPose.getOrientation().setX(data[index++]);
                kfPose.getOrientation().setY(data[index++]);
                kfPose.getOrientation().setZ(data[index++]);
                kfPose.getOrientation().setW(data[index++]);
                poseArray.getPoses().add(kfPose);

                // 添加地图点数量
                int ptsCount = (int)data[index++];
                geometry_msgs.Pose ptsCountPose = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
                ptsCountPose.getPosition().setX(ptsCount);
                ptsCountPose.getPosition().setY(ptsCount);
                ptsCountPose.getPosition().setZ(ptsCount);
                ptsCountPose.getOrientation().setW(1.0);
                poseArray.getPoses().add(ptsCountPose);

                // 添加所有地图点
                for (int j = 0; j < ptsCount; j++) {
                    geometry_msgs.Pose pointPose = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
                    pointPose.getPosition().setX(data[index++]);
                    pointPose.getPosition().setY(data[index++]);
                    pointPose.getPosition().setZ(data[index++]);
                    pointPose.getOrientation().setW(1.0);
                    poseArray.getPoses().add(pointPose);
                }
            }

            // 发布完整数据
            keyframesPublisher.publish(poseArray);
            if (DEBUG) Log.d(TAG, "Published ALL keyframe data with " + kfCount + " keyframes and their map points");

        } catch (Exception e) {
            Log.e(TAG, "Error publishing complete keyframe data: " + e.getMessage(), e);
        }
    }

    /**
     * 发布当前相机位姿方法，使用PoseStamped消息类型
     */
    public void publishCurrentCameraPose(float[] cameraPose) {
        if (!isConnected() || cameraPosePublisher == null || cameraPose == null || cameraPose.length < 7) {
            return;
        }

        try {
            geometry_msgs.PoseStamped poseStampedMsg = cameraPosePublisher.newMessage();

            // 设置Header信息
            poseStampedMsg.getHeader().setFrameId("map");
            poseStampedMsg.getHeader().setStamp(connectedNode.getCurrentTime());

            // 设置位姿信息
            poseStampedMsg.getPose().getPosition().setX(cameraPose[0]);
            poseStampedMsg.getPose().getPosition().setY(cameraPose[1]);
            poseStampedMsg.getPose().getPosition().setZ(cameraPose[2]);
            poseStampedMsg.getPose().getOrientation().setX(cameraPose[3]);
            poseStampedMsg.getPose().getOrientation().setY(cameraPose[4]);
            poseStampedMsg.getPose().getOrientation().setZ(cameraPose[5]);
            poseStampedMsg.getPose().getOrientation().setW(cameraPose[6]);

            // 发布消息
            cameraPosePublisher.publish(poseStampedMsg);
            if (DEBUG) Log.d(TAG, "Published current camera pose to /orb_slam3/camera_pose");
        } catch (Exception e) {
            Log.e(TAG, "Error publishing camera pose: " + e.getMessage(), e);
        }
    }

    /**
     * 发布网格地图到 /grid_map 话题
     */
    public void publishGridMap(byte[][] mapData, GridMapBuilder.GridMapInfo mapInfo) {
        if (!isConnected() || gridMapPublisher == null || mapData == null || mapInfo == null) {
            if (DEBUG && !isConnected()) Log.d(TAG, "ROS未连接，无法发布网格地图");
            return;
        }

        try {
            // 创建OccupancyGrid消息
            nav_msgs.OccupancyGrid gridMsg = gridMapPublisher.newMessage();

            // 设置Header
            gridMsg.getHeader().setFrameId("map");
            gridMsg.getHeader().setStamp(connectedNode.getCurrentTime());

            // 设置地图元信息
            gridMsg.getInfo().setWidth(mapInfo.width);
            gridMsg.getInfo().setHeight(mapInfo.height);
            gridMsg.getInfo().setResolution(mapInfo.resolution);

            // 设置地图原点
            gridMsg.getInfo().getOrigin().getPosition().setX(mapInfo.originX);
            gridMsg.getInfo().getOrigin().getPosition().setY(mapInfo.originY);
            gridMsg.getInfo().getOrigin().getPosition().setZ(0.0);
            gridMsg.getInfo().getOrigin().getOrientation().setX(0.0);
            gridMsg.getInfo().getOrigin().getOrientation().setY(0.0);
            gridMsg.getInfo().getOrigin().getOrientation().setZ(0.0);
            gridMsg.getInfo().getOrigin().getOrientation().setW(1.0);

            // 转换和设置地图数据
            int totalSize = mapInfo.width * mapInfo.height;

            synchronized (gridMapChannelBufferStream) {
                gridMapChannelBufferStream.buffer().clear();

                // 按照ROS OccupancyGrid的存储格式转换数据
                for (int y = 0; y < mapInfo.height; y++) {
                    for (int x = 0; x < mapInfo.width; x++) {
                        int dataY = mapInfo.height - y - 1;
                        byte occupancyValue = mapData[dataY][x];
                        gridMapChannelBufferStream.write(occupancyValue & 0xFF);
                    }
                }

                gridMsg.setData(gridMapChannelBufferStream.buffer().copy());
            }

            // 发布消息
            gridMapPublisher.publish(gridMsg);

            if (DEBUG) {
                Log.d(TAG, String.format("✅ 发布网格地图成功到/grid_map: %dx%d, 分辨率: %.3f m/pix, 数据大小: %d",
                        mapInfo.width, mapInfo.height, mapInfo.resolution, totalSize));
            }

        } catch (Exception e) {
            Log.e(TAG, "发布网格地图时出错: " + e.getMessage(), e);
            handlePublishError(e);
        }
    }

    /**
     * 发布静态地图到 /map 话题
     */
    public void publishStaticMap(byte[][] mapData, GridMapBuilder.GridMapInfo mapInfo) {
        if (!isConnected() || staticMapPublisher == null || mapData == null || mapInfo == null) {
            if (DEBUG && !isConnected()) Log.d(TAG, "ROS未连接，无法发布静态地图");
            return;
        }

        try {
            // 创建OccupancyGrid消息
            nav_msgs.OccupancyGrid gridMsg = staticMapPublisher.newMessage();

            // 设置Header
            gridMsg.getHeader().setFrameId("map");
            gridMsg.getHeader().setStamp(connectedNode.getCurrentTime());

            // 设置地图元信息
            gridMsg.getInfo().setWidth(mapInfo.width);
            gridMsg.getInfo().setHeight(mapInfo.height);
            gridMsg.getInfo().setResolution(mapInfo.resolution);

            // 设置地图原点
            gridMsg.getInfo().getOrigin().getPosition().setX(mapInfo.originX);
            gridMsg.getInfo().getOrigin().getPosition().setY(mapInfo.originY);
            gridMsg.getInfo().getOrigin().getPosition().setZ(0.0);
            gridMsg.getInfo().getOrigin().getOrientation().setX(0.0);
            gridMsg.getInfo().getOrigin().getOrientation().setY(0.0);
            gridMsg.getInfo().getOrigin().getOrientation().setZ(0.0);
            gridMsg.getInfo().getOrigin().getOrientation().setW(1.0);

            // 转换和设置地图数据
            int totalSize = mapInfo.width * mapInfo.height;

            synchronized (gridMapChannelBufferStream) {
                gridMapChannelBufferStream.buffer().clear();

                // 按照ROS OccupancyGrid的存储格式转换数据
                for (int y = 0; y < mapInfo.height; y++) {
                    for (int x = 0; x < mapInfo.width; x++) {
                        int dataY = mapInfo.height - y - 1;
                        byte occupancyValue = mapData[dataY][x];
                        gridMapChannelBufferStream.write(occupancyValue & 0xFF);
                    }
                }

                gridMsg.setData(gridMapChannelBufferStream.buffer().copy());
            }

            // 发布消息
            staticMapPublisher.publish(gridMsg);

            if (DEBUG) {
                Log.d(TAG, String.format("✅ 发布静态地图成功到/map: %dx%d, 分辨率: %.3f m/pix, 数据大小: %d",
                        mapInfo.width, mapInfo.height, mapInfo.resolution, totalSize));
            }

        } catch (Exception e) {
            Log.e(TAG, "发布静态地图时出错: " + e.getMessage(), e);
            handlePublishError(e);
        }
    }

    // ===== 深度图发布方法 =====

    /**
     * 发布深度图到ROS话题
     * @param depthMatAddr 深度图的native地址
     */
    public void publishDepthImage(long depthMatAddr) {
        try {
            // 检查发布器订阅者数量
            int subscriberCount = depthImagePublisher.getNumberOfSubscribers();

            // 获取深度图数据和尺寸
            float[] depthData = getDepthDataFromNative(depthMatAddr);
            int[] dimensions = getDepthDimensionsFromNative(depthMatAddr);

            if (depthData == null || dimensions == null || dimensions.length < 2) {
                String debugMsg = "❌ 深度图数据获取失败: ";
                if (depthData == null) debugMsg += "数据为空 ";
                if (dimensions == null) debugMsg += "尺寸为空 ";
                if (dimensions != null && dimensions.length < 2) debugMsg += "尺寸数据不足 ";
                updateDebugInfo(debugMsg);
                return;
            }

            int width = dimensions[0];
            int height = dimensions[1];
            int validPixels = countValidDepth(depthData);

            // 验证数据完整性
            if (width <= 0 || height <= 0) {
                return;
            }

            if (depthData.length != width * height) {
                return;
            }

            // 详细分析数据分布
            float minDepth = Float.MAX_VALUE;
            float maxDepth = Float.MIN_VALUE;
            float avgDepth = 0;
            int firstValidIndex = -1;

            for (int i = 0; i < depthData.length; i++) {
                if (depthData[i] > 0) {
                    if (firstValidIndex == -1) firstValidIndex = i;
                    if (depthData[i] < minDepth) minDepth = depthData[i];
                    if (depthData[i] > maxDepth) maxDepth = depthData[i];
                    avgDepth += depthData[i];
                }
            }

            if (validPixels > 0) {
                avgDepth /= validPixels;
            } else {
                return;
            }

            // 创建ROS Image消息
            sensor_msgs.Image depthMsg = depthImagePublisher.newMessage();

            // 设置Header
            depthMsg.getHeader().setFrameId("camera_left");
            depthMsg.getHeader().setStamp(connectedNode.getCurrentTime());

            // 设置图像信息
            depthMsg.setWidth(width);
            depthMsg.setHeight(height);
            depthMsg.setEncoding("32FC1"); // 32位浮点数，单通道
            depthMsg.setIsBigendian((byte)0); // 小端序
            depthMsg.setStep(width * 4); // 每行字节数：width * 4 bytes per float

            // 转换float数组为byte数组
            byte[] imageData = new byte[depthData.length * 4];
            ByteBuffer buffer = ByteBuffer.wrap(imageData);
            buffer.order(ByteOrder.LITTLE_ENDIAN);

            for (float depth : depthData) {
                buffer.putFloat(depth);
            }

            // 设置数据
            ChannelBufferOutputStream stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
            stream.write(imageData);
            depthMsg.setData(stream.buffer());

            try {
                depthImagePublisher.publish(depthMsg);
                try {
                    Thread.sleep(50);
                    int newSubscriberCount = depthImagePublisher.getNumberOfSubscribers();
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                }
            } catch (Exception publishEx) {
                Log.e(TAG, "发布深度图失败: " + publishEx.getMessage(), publishEx);
            }
        } catch (Exception e) {
            Log.e(TAG, "处理深度图时出错: " + e.getMessage(), e);
        }
    }

    /**
     * 检查深度图发布器是否有订阅者
     * @return 是否有订阅者
     */
    public boolean hasDepthImageSubscribers() {
        if (depthImagePublisher == null) return false;
        try {
            return depthImagePublisher.getNumberOfSubscribers() > 0;
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * 统计有效深度像素数量
     */
    private int countValidDepth(float[] depthData) {
        int count = 0;
        for (float depth : depthData) {
            if (depth > 0) count++;
        }
        return count;
    }

    /**
     * 更新调试信息
     */
    private void updateDebugInfo(String message) {
        if (debugLogger != null) {
            debugLogger.updateDebugTextView(message);
        }
    }

    // ===== 发布器状态检查方法 =====

    public boolean isPtsAndPosePublisherReady() {
        return mapPointsPublisher != null;
    }

    public boolean isAllKfAndPtsPublisherReady() {
        return keyframesPublisher != null;
    }

    public boolean isCurCameraPosePublisherReady() {
        return cameraPosePublisher != null;
    }

    public boolean isGridMapPublisherReady() {
        return gridMapPublisher != null;
    }

    public boolean isStaticMapPublisherReady() {
        return staticMapPublisher != null;
    }

    // ===== JNI方法声明 =====

    /**
     * 从native地址获取深度数据
     */
    private native float[] getDepthDataFromNative(long depthMatAddr);

    /**
     * 从native地址获取深度图尺寸
     */
    private native int[] getDepthDimensionsFromNative(long depthMatAddr);

    // ===== 错误处理方法 =====

    /**
     * 处理发布错误
     */
    private void handlePublishError(Exception e) {
        long currentTime = System.currentTimeMillis();

        errorCount++;

        if (errorCount == 1 || (currentTime - lastErrorTime > 5000)) {
            Log.e(TAG, "发布消息时出错: " + e.getMessage(), e);
            lastErrorTime = currentTime;
        }

        if (errorCount >= MAX_ERROR_COUNT) {
            Log.e(TAG, "连续发布错误次数过多，尝试重新初始化ROS连接");
            errorCount = 0;
            reinitializeConnection();
        }
    }

    /**
     * 尝试重新初始化连接
     */
    private void reinitializeConnection() {
        shutdown();

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        initialize();
    }

    /**
     * 关闭ROS连接并释放资源
     */
    public void shutdown() {
        if (nodeMainExecutor != null) {
            try {
                nodeMainExecutor.shutdown();
                nodeMainExecutor = null;
            } catch (Exception e) {
                Log.e(TAG, "关闭ROS节点时出错: " + e.getMessage(), e);
            }
        }

        // 重置所有发布器引用
        synchronized (this) {
            leftCompressedImagePublisher = null;
            rightCompressedImagePublisher = null;
            cameraPosePublisher = null;
            mapPointsPublisher = null;
            keyframesPublisher = null;
            gridMapPublisher = null;
            staticMapPublisher = null;
            depthImagePublisher = null;
            connectedNode = null;
        }

        isConnected.set(false);

        if (DEBUG) Log.i(TAG, "ROS连接已关闭");
    }

    // ===== 工具方法 =====

    /**
     * 在UI线程上显示Toast消息
     */
    private void showToast(final String message) {
        if (context != null) {
            try {
                ((VslamActivity) context).runOnUiThread(() ->
                        Toast.makeText(context, message, Toast.LENGTH_SHORT).show());
            } catch (Exception e) {
                Log.e(TAG, "显示Toast消息失败: " + e.getMessage());
            }
        }
    }

    /**
     * 获取本地IP地址
     */
    private InetAddress getLocalIPAddress() {
        try {
            InetAddress wifiAddress = getWifiIPAddress();
            if (wifiAddress != null) {
                Log.i(TAG, "使用WiFi IP地址: " + wifiAddress.getHostAddress());
                return wifiAddress;
            }

            for (Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces();
                 en.hasMoreElements();) {
                NetworkInterface intf = en.nextElement();

                if (intf.isLoopback() || !intf.isUp() || intf.isVirtual()) {
                    continue;
                }

                for (Enumeration<InetAddress> enumIpAddr = intf.getInetAddresses();
                     enumIpAddr.hasMoreElements();) {
                    InetAddress inetAddress = enumIpAddr.nextElement();
                    if (!inetAddress.isLoopbackAddress() &&
                            inetAddress instanceof Inet4Address) {
                        Log.i(TAG, "使用网络接口[" + intf.getName() + "] IP地址: " +
                                inetAddress.getHostAddress());
                        return inetAddress;
                    }
                }
            }

            Log.w(TAG, "无法找到合适的网络接口，使用回环地址");
            return InetAddress.getLocalHost();

        } catch (Exception e) {
            Log.e(TAG, "获取本地IP地址时出错: " + e.getMessage(), e);
        }
        return null;
    }

    /**
     * 获取WiFi接口的IP地址
     */
    private InetAddress getWifiIPAddress() {
        try {
            for (Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces();
                 en.hasMoreElements();) {
                NetworkInterface intf = en.nextElement();

                String name = intf.getName();
                if ((name.contains("wlan") || name.contains("eth") || name.contains("ap")) &&
                        !intf.isLoopback() && intf.isUp()) {
                    for (Enumeration<InetAddress> enumIpAddr = intf.getInetAddresses();
                         enumIpAddr.hasMoreElements();) {
                        InetAddress inetAddress = enumIpAddr.nextElement();
                        if (!inetAddress.isLoopbackAddress() &&
                                inetAddress instanceof Inet4Address) {
                            return inetAddress;
                        }
                    }
                }
            }
        } catch (Exception e) {
            Log.e(TAG, "获取WiFi IP地址时出错: " + e.getMessage(), e);
        }
        return null;
    }

    /**
     * 测试ChannelBuffer能否正常工作
     */
    private void testChannelBuffer() {
        try {
            if (leftCompressedImagePublisher != null) {
                CompressedImage testMsg = leftCompressedImagePublisher.newMessage();
                testMsg.setFormat("jpeg");
                testMsg.getHeader().setFrameId("test_frame");
                testMsg.getHeader().setStamp(connectedNode.getCurrentTime());

                synchronized (leftChannelBufferStream) {
                    leftChannelBufferStream.buffer().clear();
                    leftChannelBufferStream.write(new byte[]{(byte)0xFF, (byte)0xD8, (byte)0xFF, (byte)0xD9});
                    testMsg.setData(leftChannelBufferStream.buffer().copy());
                }

                leftCompressedImagePublisher.publish(testMsg);
                Log.i(TAG, "测试缓冲区成功发布");
            }
        } catch (Exception e) {
            Log.e(TAG, "测试缓冲区失败: " + e.getMessage(), e);
        }
    }

    /**
     * 获取ConnectedNode对象（用于高级操作）
     */
    public ConnectedNode getConnectedNode() {
        return connectedNode;
    }

    // ===== NodeMain接口的实现 =====

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("android/stereo_camera");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        try {
            // 初始化DebugLogger
            this.debugLogger = new DebugLogger(context);

            // 创建图像发布器
            leftCompressedImagePublisher = connectedNode.newPublisher(
                    "/stereo/left/compressed", CompressedImage._TYPE);
            rightCompressedImagePublisher = connectedNode.newPublisher(
                    "/stereo/right/compressed", CompressedImage._TYPE);
            mapsCountPublisher = connectedNode.newPublisher("/orb_slam3/maps_count", Int32._TYPE);

            // 创建SLAM数据发布器
            mapPointsPublisher = connectedNode.newPublisher(
                    "/pts_and_pose", geometry_msgs.PoseArray._TYPE);
            keyframesPublisher = connectedNode.newPublisher(
                    "/all_kf_and_pts", geometry_msgs.PoseArray._TYPE);
            cameraPosePublisher = connectedNode.newPublisher(
                    "/orb_slam3/camera_pose", geometry_msgs.PoseStamped._TYPE);

            // 创建地图发布器
            gridMapPublisher = connectedNode.newPublisher(
                    "/grid_map", nav_msgs.OccupancyGrid._TYPE);      // 实时SLAM地图
            staticMapPublisher = connectedNode.newPublisher(
                    "/map", nav_msgs.OccupancyGrid._TYPE);           // 静态预制地图

            // 创建深度图发布器
            depthImagePublisher = connectedNode.newPublisher(
                    "/stereo/depth", sensor_msgs.Image._TYPE);

            Log.i(TAG, "ROS发布器已创建成功");
            //showToast("ROS发布器已创建");

            // 验证所有发布器
            if (leftCompressedImagePublisher != null &&
                    rightCompressedImagePublisher != null &&
                    cameraPosePublisher != null &&
                    mapPointsPublisher != null &&
                    keyframesPublisher != null &&
                    gridMapPublisher != null &&
                    staticMapPublisher != null &&
                    depthImagePublisher != null) {

                isConnected.set(true);

                testChannelBuffer();
            } else {
                String failureInfo = "部分发布器创建失败：" +
                        "left=" + (leftCompressedImagePublisher != null) +
                        ", right=" + (rightCompressedImagePublisher != null) +
                        ", camera_pose=" + (cameraPosePublisher != null) +
                        ", map_points=" + (mapPointsPublisher != null) +
                        ", keyframes=" + (keyframesPublisher != null) +
                        ", grid_map=" + (gridMapPublisher != null) +
                        ", static_map=" + (staticMapPublisher != null) +
                        ", depth=" + (depthImagePublisher != null);
                Log.e(TAG, failureInfo);
                if (debugLogger != null) {
                    debugLogger.updateDebugTextView("❌ " + failureInfo);
                }
            }

        } catch (Exception e) {
            Log.e(TAG, "创建ROS发布器时出错: " + e.getMessage(), e);
            if (debugLogger != null) {
                debugLogger.updateDebugTextView("❌ 创建ROS发布器失败: " + e.getMessage());
            }
            //showToast("创建ROS发布器失败: " + e.getMessage());
        }
    }

    @Override
    public void onShutdown(Node node) {
        Log.i(TAG, "ROS节点已关闭");
        isConnected.set(false);

        // 清除所有发布器引用
        synchronized (this) {
            leftCompressedImagePublisher = null;
            rightCompressedImagePublisher = null;
            mapPointsPublisher = null;
            keyframesPublisher = null;
            cameraPosePublisher = null;
            gridMapPublisher = null;
            staticMapPublisher = null;
            depthImagePublisher = null;
            connectedNode = null;
        }
    }

    @Override
    public void onShutdownComplete(Node node) {
        Log.i(TAG, "ROS节点完全关闭");
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        Log.e(TAG, "ROS节点错误: " + throwable.getMessage(), throwable);

        throwable.printStackTrace();

        isConnected.set(false);
        //showToast("ROS节点出错: " + throwable.getMessage());

        handlePublishError(new Exception(throwable));
    }
}