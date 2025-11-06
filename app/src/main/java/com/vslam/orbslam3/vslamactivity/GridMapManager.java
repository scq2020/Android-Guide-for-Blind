package com.vslam.orbslam3.vslamactivity;

import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.Context;
import android.os.Handler;
import android.os.Looper;
import android.text.InputType;
import android.util.Log;
import android.widget.EditText;
import android.widget.Toast;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

import geometry_msgs.PoseArray;

/**
 * 网格地图管理器 - 负责网格地图的构建、ROS集成、保存和UI管理
 */
public class GridMapManager implements NodeMain {
    private static final String TAG = "GridMapManager";
    private static final boolean DEBUG = true;

    // 核心组件
    private final VslamActivity activity;
    private final RosManager rosManager;
    private GridMapBuilder gridMapBuilder;

    // ROS相关
    private ConnectedNode connectedNode;
    private Subscriber<PoseArray> ptsAndPoseSubscriber;
    private Subscriber<PoseArray> allKfAndPtsSubscriber;
    private final AtomicBoolean isRosSubscriptionActive = new AtomicBoolean(false);

    // 状态管理
    private final Handler mainHandler;
    private final ExecutorService backgroundExecutor;
    private boolean isInitialized = false;
    private boolean autoSaveEnabled = false;
    private long lastSaveTime = 0;
    private static final long AUTO_SAVE_INTERVAL = 30000; // 30秒自动保存间隔

    // 文件管理
    private String mapSaveDirectory;
    private static final String MAP_FILE_PREFIX = "grid_map_";

    /**
     * 构造函数
     */
    public GridMapManager(VslamActivity activity, RosManager rosManager) {
        this.activity = activity;
        this.rosManager = rosManager;
        this.mainHandler = new Handler(Looper.getMainLooper());
        this.backgroundExecutor = Executors.newSingleThreadExecutor();

        initialize();
    }

    /**
     * 初始化网格地图管理器
     */
    private void initialize() {
        try {
            // 创建网格地图构建器
            gridMapBuilder = new GridMapBuilder();

            // 设置地图保存目录
            setupMapSaveDirectory();

            isInitialized = true;

            if (DEBUG) {
                Log.i(TAG, "网格地图管理器初始化完成");
            }

        } catch (Exception e) {
            Log.e(TAG, "网格地图管理器初始化失败: " + e.getMessage(), e);
            //showToast("网格地图管理器初始化失败");
        }
    }

    /**
     * 设置地图保存目录
     */
    private void setupMapSaveDirectory() {
        File externalDir = activity.getExternalFilesDir(null);
        if (externalDir == null) {
            externalDir = activity.getFilesDir();
        }

        mapSaveDirectory = new File(externalDir, "SLAM/Maps").getAbsolutePath();
        File mapDir = new File(mapSaveDirectory);
        if (!mapDir.exists()) {
            mapDir.mkdirs();
        }

        if (DEBUG) {
            Log.i(TAG, "地图保存目录: " + mapSaveDirectory);
        }
    }

    // ===== 地图保存功能 =====

    /**
     * 保存地图（同时保存网格地图和.osa文件）
     */
    public void saveMap() {
        if (!isInitialized) {
            //showToast("网格地图未初始化");
            return;
        }

        // 创建进度对话框
        ProgressDialog progressDialog = new ProgressDialog(activity);
        progressDialog.setTitle("保存地图");
        progressDialog.setMessage("正在保存地图文件...");
        progressDialog.setCancelable(false);
        progressDialog.show();

        // 后台保存
        backgroundExecutor.execute(() -> {
            boolean success = false;
            String errorMessage = "";

            try {
                // 1. 保存网格地图
                saveGridMapInternal();
                Log.i(TAG, "网格地图保存成功");

                // 2. 保存OSA地图
                if (activity.isSlamInitialized()) {
                    File externalDir = activity.getExternalFilesDir(null);
                    if (externalDir == null) {
                        externalDir = activity.getFilesDir();
                    }

                    String osaFilePath = new File(externalDir, "SLAM/MapData/new_workspace").getAbsolutePath();

                    // 确保目录存在
                    new File(externalDir, "SLAM/MapData").mkdirs();

                    boolean saveResult = activity.saveMapOSA(osaFilePath);
                    if (saveResult) {
                        Log.i(TAG, "OSA地图保存成功");
                        success = true;
                    } else {
                        throw new Exception("OSA地图保存失败");
                    }
                } else {
                    throw new Exception("SLAM未初始化，无法保存地图");
                }

            } catch (Exception e) {
                Log.e(TAG, "保存地图失败", e);
                errorMessage = e.getMessage();
            }

            // 显示结果
            final boolean finalSuccess = success;
            final String finalErrorMessage = errorMessage;

            mainHandler.post(() -> {
                progressDialog.dismiss();
                if (finalSuccess) {
                    new AlertDialog.Builder(activity)
                            .setTitle("地图保存成功")
                            .setMessage("地图文件已保存完成")
                            .setPositiveButton("确定", null)
                            .show();
                } else {
                    new AlertDialog.Builder(activity)
                            .setTitle("地图保存失败")
                            .setMessage("错误信息: " + finalErrorMessage)
                            .setPositiveButton("确定", null)
                            .show();
                }
            });
        });
    }

    /**
     * 快速保存网格地图（不包括OSA）
     */
    public void saveGridMapOnly() {
        if (!isInitialized) {
            //showToast("网格地图未初始化");
            return;
        }

        backgroundExecutor.execute(() -> {
            try {
                saveGridMapInternal();
                mainHandler.post(() -> showToast("网格地图保存完成"));
            } catch (Exception e) {
                Log.e(TAG, "保存网格地图失败", e);
                mainHandler.post(() -> showToast("网格地图保存失败: " + e.getMessage()));
            }
        });
    }

    /**
     * 内部保存网格地图方法
     */
    private void saveGridMapInternal() {
        if (!isInitialized || gridMapBuilder == null) {
            Log.w(TAG, "网格地图未初始化，无法保存");
            return;
        }

        try {
            byte[][] mapData = gridMapBuilder.getMapData();
            GridMapBuilder.GridMapInfo mapInfo = gridMapBuilder.getMapInfo();

            if (mapData == null || mapInfo == null) {
                Log.w(TAG, "地图数据为空，无法保存");
                return;
            }

            // 确保保存目录存在
            File mapDir = new File(mapSaveDirectory);
            if (!mapDir.exists()) {
                boolean created = mapDir.mkdirs();
                if (created) {
                    Log.i(TAG, "创建地图保存目录: " + mapSaveDirectory);
                } else {
                    Log.e(TAG, "无法创建地图保存目录: " + mapSaveDirectory);
                    return;
                }
            }

            // 使用固定文件名，覆盖之前的文件
            String mapName = "new_map";

            // 保存为PGM格式
            savePGMMap(mapData, mapInfo, mapName);

            // 保存为YAML元数据文件
            saveYAMLMetadata(mapInfo, mapName);

            lastSaveTime = System.currentTimeMillis();

            if (DEBUG) {
                Log.i(TAG, "网格地图保存完成: " + mapName);
            }

        } catch (Exception e) {
            Log.e(TAG, "保存网格地图时出错: " + e.getMessage(), e);
            throw new RuntimeException("保存网格地图失败: " + e.getMessage());
        }
    }

    /**
     * 保存PGM格式地图文件
     */
    private void savePGMMap(byte[][] mapData, GridMapBuilder.GridMapInfo mapInfo, String mapName)
            throws IOException {
        File pgmFile = new File(mapSaveDirectory, mapName + ".pgm");
        FileOutputStream fos = new FileOutputStream(pgmFile);

        try {
            // 写入PGM文件头
            String header = String.format("P5\n# CREATOR: GridMapManager %.3f m/pix\n%d %d\n255\n",
                    mapInfo.resolution, mapInfo.width, mapInfo.height);
            fos.write(header.getBytes());

            // ✅ 修复：直接按照原始顺序写入，不进行Y轴翻转
            for (int y = 0; y < mapInfo.height; y++) {
                for (int x = 0; x < mapInfo.width; x++) {
                    // ✅ 直接使用原始坐标，不翻转
                    int occupancyValue = mapData[y][x] & 0xFF;

                    int grayValue;
                    if (occupancyValue <= 25) {
                        grayValue = 254; // 自由空间 -> 白色
                    } else if (occupancyValue >= 65) {
                        grayValue = 0;   // 占用空间 -> 黑色
                    } else {
                        grayValue = 205; // 未知空间 -> 灰色
                    }

                    fos.write(grayValue);
                }
            }
        } finally {
            fos.close();
        }
    }

    /**
     * 保存YAML元数据文件
     */
    private void saveYAMLMetadata(GridMapBuilder.GridMapInfo mapInfo, String mapName)
            throws IOException {
        File yamlFile = new File(mapSaveDirectory, mapName + ".yaml");
        FileOutputStream fos = new FileOutputStream(yamlFile);

        try {
            String yamlContent = String.format(
                    "image: %s.pgm\n" +
                            "resolution: %f\n" +
                            "origin: [%f, %f, %f]\n" +
                            "negate: 0\n" +
                            "occupied_thresh: 0.65\n" +
                            "free_thresh: 0.196\n\n",
                    mapName, mapInfo.resolution,
                    mapInfo.originX, mapInfo.originY, 0.0
            );

            fos.write(yamlContent.getBytes());

        } finally {
            fos.close();
        }
    }

    // ===== 地图设置对话框 =====

    /**
     * 显示网格地图设置对话框
     */
    public void showGridMapSettings() {
        if (!isInitialized) {
            //showToast("网格地图管理器未初始化");
            return;
        }

        AlertDialog.Builder builder = new AlertDialog.Builder(activity);
        builder.setTitle("网格地图设置");

        String[] options = {
                "保存完整地图 (网格+OSA)",
                "仅保存网格地图",
                "启用/禁用自动保存 (" + (autoSaveEnabled ? "已启用" : "已禁用") + ")",
                "设置自动保存间隔",
                "查看地图信息",
                "检查ROS订阅状态",
                "重置地图数据",
                "网格地图状态",
                "清理地图文件"
        };

        builder.setItems(options, (dialog, which) -> {
            switch (which) {
                case 0:
                    saveMap();
                    break;
                case 1:
                    saveGridMapOnly();
                    break;
                case 2:
                    toggleAutoSave();
                    break;
                case 3:
                    showAutoSaveIntervalDialog();
                    break;
                case 4:
                    showMapInfo();
                    break;
                case 5:
                    checkRosSubscriptionStatus();
                    break;
                case 6:
                    resetMapData();
                    break;
                case 7:
                    showGridMapStatus();
                    break;
                case 8:
                    cleanupMapFiles();
                    break;
            }
        });

        builder.setNegativeButton("取消", null);
        builder.show();
    }

    /**
     * 切换自动保存状态
     */
    private void toggleAutoSave() {
        setAutoSave(!autoSaveEnabled);
        //showToast("自动保存已" + (autoSaveEnabled ? "启用" : "禁用"));
    }

    /**
     * 显示自动保存间隔设置对话框
     */
    private void showAutoSaveIntervalDialog() {
        AlertDialog.Builder builder = new AlertDialog.Builder(activity);
        builder.setTitle("自动保存间隔");

        final EditText input = new EditText(activity);
        input.setInputType(InputType.TYPE_CLASS_NUMBER);
        input.setText(String.valueOf(AUTO_SAVE_INTERVAL / 1000));
        input.setHint("秒");
        builder.setView(input);

        builder.setPositiveButton("确定", (dialog, which) -> {
            try {
                int seconds = Integer.parseInt(input.getText().toString());
                if (seconds > 0) {
                    // 这里可以添加动态修改间隔的逻辑
                    //showToast("自动保存间隔设置为: " + seconds + "秒");
                } else {
                    //showToast("间隔必须大于0");
                }
            } catch (NumberFormatException e) {
               // showToast("输入格式错误");
            }
        });

        builder.setNegativeButton("取消", null);
        builder.show();
    }

    /**
     * 显示地图信息
     */
    public void showMapInfo() {
        GridMapBuilder.GridMapInfo mapInfo = getMapInfo();
        if (mapInfo != null) {
            String info = String.format(
                    "地图信息:\n" +
                            "尺寸: %dx%d\n" +
                            "分辨率: %.3f m/像素\n" +
                            "原点: (%.2f, %.2f)\n" +
                            "更新时间: %s\n" +
                            "自动保存: %s\n" +
                            "地图文件: %s\n" +
                            "状态: %s",
                    mapInfo.width, mapInfo.height,
                    mapInfo.resolution,
                    mapInfo.originX, mapInfo.originY,
                    new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault()).format(new Date(mapInfo.timestamp)),
                    autoSaveEnabled ? "启用" : "禁用",
                    mapSaveDirectory,
                    "活跃"
            );

            new AlertDialog.Builder(activity)
                    .setTitle("地图信息")
                    .setMessage(info)
                    .setPositiveButton("确定", null)
                    .setNeutralButton("打开地图目录", (dialog, which) -> {
                        // 这里可以添加打开文件管理器的逻辑
                        //showToast("地图目录: " + mapSaveDirectory);
                    })
                    .show();
        } else {
            //showToast("地图信息不可用");
        }
    }

    /**
     * 显示网格地图状态
     */
    public void showGridMapStatus() {
        StringBuilder status = new StringBuilder();
        status.append("网格地图状态:\n");
        status.append("管理器状态: ").append(isInitialized ? "已初始化" : "未初始化").append("\n");
        status.append("地图信息: ").append(getMapInfo() != null ? "可用" : "不可用").append("\n");
        status.append("ROS连接: ").append(rosManager != null && rosManager.isConnected() ? "已连接" : "未连接").append("\n");
        status.append("ROS订阅: ").append(isRosSubscriptionActive() ? "活跃" : "非活跃").append("\n");
        status.append("自动保存: ").append(autoSaveEnabled ? "启用" : "禁用").append("\n");
        status.append("保存目录: ").append(mapSaveDirectory).append("\n");
        status.append("最后保存: ").append(lastSaveTime > 0 ?
                new SimpleDateFormat("HH:mm:ss", Locale.getDefault()).format(new Date(lastSaveTime)) : "无").append("\n");

        new AlertDialog.Builder(activity)
                .setTitle("网格地图状态")
                .setMessage(status.toString())
                .setPositiveButton("确定", null)
                .setNeutralButton("刷新", (dialog, which) -> showGridMapStatus())
                .show();
    }

    /**
     * 检查ROS订阅状态
     */
    public void checkRosSubscriptionStatus() {
        boolean rosConnected = rosManager != null && rosManager.isConnected();
        boolean rosSubscriptionActive = isRosSubscriptionActive();

        String status = String.format(
                "ROS状态:\n" +
                        "ROS连接: %s\n" +
                        "网格地图ROS订阅: %s\n" +
                        "发布器状态: %s\n" +
                        "pts_and_pose订阅: %s\n" +
                        "all_kf_and_pts订阅: %s\n" +
                        "ROS发布中: %s",
                rosConnected ? "已连接" : "未连接",
                rosSubscriptionActive ? "活跃" : "非活跃",
                rosManager != null && rosManager.isGridMapPublisherReady() ? "就绪" : "未就绪",
                ptsAndPoseSubscriber != null ? "活跃" : "未活跃",
                allKfAndPtsSubscriber != null ? "活跃" : "未活跃",
                activity.isRosPublishing() ? "是" : "否"
        );

        new AlertDialog.Builder(activity)
                .setTitle("ROS订阅状态")
                .setMessage(status)
                .setPositiveButton("确定", null)
                .setNeutralButton("重新连接ROS", (dialog, which) -> {
                    if (rosManager != null) {
                        backgroundExecutor.execute(() -> {
                            try {
                                // 重新启动ROS订阅
                                shutdown();
                                // 等待一段时间后重新初始化
                                Thread.sleep(2000);
                                initialize();

                                mainHandler.post(() -> {
                                    //showToast("ROS重新连接完成");
                                });
                            } catch (Exception e) {
                                Log.e(TAG, "重新连接ROS失败", e);
                                mainHandler.post(() -> {
                                   // showToast("重新连接失败: " + e.getMessage());
                                });
                            }
                        });
                    }
                })
                .show();
    }

    /**
     * 重置地图数据
     */
    private void resetMapData() {
        new AlertDialog.Builder(activity)
                .setTitle("重置地图数据")
                .setMessage("这将清除所有地图数据，确定继续吗？")
                .setPositiveButton("确定", (dialog, which) -> {
                    backgroundExecutor.execute(() -> {
                        try {
                            // 重新创建地图构建器
                            if (gridMapBuilder != null) {
                                gridMapBuilder = new GridMapBuilder();
                            }

                            lastSaveTime = 0;

                            mainHandler.post(() -> {
                               // showToast("地图数据已重置");
                            });
                        } catch (Exception e) {
                            Log.e(TAG, "重置地图数据失败", e);
                            mainHandler.post(() -> {
                               // showToast("重置失败: " + e.getMessage());
                            });
                        }
                    });
                })
                .setNegativeButton("取消", null)
                .show();
    }

    /**
     * 清理地图文件
     */
    private void cleanupMapFiles() {
        new AlertDialog.Builder(activity)
                .setTitle("清理地图文件")
                .setMessage("这将删除所有保存的地图文件，确定继续吗？")
                .setPositiveButton("确定", (dialog, which) -> {
                    backgroundExecutor.execute(() -> {
                        try {
                            File mapDir = new File(mapSaveDirectory);
                            if (mapDir.exists()) {
                                File[] files = mapDir.listFiles();
                                if (files != null) {
                                    int deletedCount = 0;
                                    for (File file : files) {
                                        if (file.isFile() && (file.getName().endsWith(".pgm") ||
                                                file.getName().endsWith(".yaml"))) {
                                            if (file.delete()) {
                                                deletedCount++;
                                            }
                                        }
                                    }

                                    final int finalDeletedCount = deletedCount;
                                    mainHandler.post(() -> {
                                       // showToast("已删除 " + finalDeletedCount + " 个地图文件");
                                    });
                                } else {
                                    mainHandler.post(() -> {
                                       // showToast("地图目录为空");
                                    });
                                }
                            } else {
                                mainHandler.post(() -> {
                                   // showToast("地图目录不存在");
                                });
                            }
                        } catch (Exception e) {
                            Log.e(TAG, "清理地图文件失败", e);
                            mainHandler.post(() -> {
                               // showToast("清理失败: " + e.getMessage());
                            });
                        }
                    });
                })
                .setNegativeButton("取消", null)
                .show();
    }

    // ===== 原有的核心功能方法 =====

    /**
     * 处理关键帧数据
     */
    public void processKeyframeData(float[] keyFrameData) {
        if (!isInitialized || keyFrameData == null || keyFrameData.length < 7) {
            if (DEBUG) Log.w(TAG, "关键帧数据无效，长度: " +
                    (keyFrameData != null ? keyFrameData.length : 0));
            return;
        }

        try {
            if (activity.isNavigationMode()) {
                Log.i(TAG, "导航模式：跳过地图构建");
                return;
            }
            // 使用GridMapBuilder更新地图
            gridMapBuilder.updateGridMap(keyFrameData);

            // 获取更新后的地图数据
            byte[][] mapData = gridMapBuilder.getMapData();
            GridMapBuilder.GridMapInfo mapInfo = gridMapBuilder.getMapInfo();

            if (mapData != null && mapInfo != null) {
                // 发布到ROS
                if (rosManager != null && rosManager.isGridMapPublisherReady()) {
                    rosManager.publishGridMap(mapData, mapInfo);

                    if (DEBUG) {
                        Log.d(TAG, String.format("处理关键帧数据并发布网格地图: %d个地图点",
                                (keyFrameData.length - 7) / 3));
                    }
                }

                // 更新UI显示
                if (activity != null && activity.uiHelper != null) {
                    activity.runOnUI(() -> {
                        activity.uiHelper.updateGridMap(mapData, mapInfo);
                    });
                }

                // 检查自动保存
                checkAutoSave();
            }

        } catch (Exception e) {
            Log.e(TAG, "处理关键帧数据时出错: " + e.getMessage(), e);
        }
    }

    /**
     * 处理所有关键帧数据（闭环重建）
     */
    public void processAllKeyframesData(float[] allKeyframesData) {
        if (!isInitialized || allKeyframesData == null || allKeyframesData.length < 1) {
            Log.w(TAG, "所有关键帧数据无效");
            return;
        }

        try {
            if (DEBUG) {
                Log.i(TAG, "开始重建网格地图，数据长度: " + allKeyframesData.length);
            }
            if (activity.isNavigationMode()) {
                Log.i(TAG, "导航模式：跳过地图重建");
                return;
            }
            // 使用GridMapBuilder重置并重建地图
            gridMapBuilder.resetGridMap(allKeyframesData);

            // 获取重建后的地图数据
            byte[][] mapData = gridMapBuilder.getMapData();
            GridMapBuilder.GridMapInfo mapInfo = gridMapBuilder.getMapInfo();

            if (mapData != null && mapInfo != null) {
                // 发布到ROS
                if (rosManager != null && rosManager.isGridMapPublisherReady()) {
                    rosManager.publishGridMap(mapData, mapInfo);

                    if (DEBUG) {
                        Log.i(TAG, "网格地图重建完成并发布");
                    }
                }

                // 重建后自动保存
                if (autoSaveEnabled) {
                    backgroundExecutor.execute(() -> {
                        try {
                            saveGridMapInternal();
                        } catch (Exception e) {
                            Log.e(TAG, "重建后自动保存失败", e);
                        }
                    });
                }
            }

        } catch (Exception e) {
            Log.e(TAG, "处理所有关键帧数据时出错: " + e.getMessage(), e);
        }
    }

    /**
     * 检查自动保存
     */
    private void checkAutoSave() {
        if (!autoSaveEnabled) return;

        long currentTime = System.currentTimeMillis();
        if (currentTime - lastSaveTime > AUTO_SAVE_INTERVAL) {
            backgroundExecutor.execute(() -> {
                try {
                    saveGridMapInternal();
                } catch (Exception e) {
                    Log.e(TAG, "自动保存失败", e);
                }
            });
        }
    }

    /**
     * 获取地图信息
     */
    public GridMapBuilder.GridMapInfo getMapInfo() {
        if (!isInitialized || gridMapBuilder == null) {
            return null;
        }
        return gridMapBuilder.getMapInfo();
    }

    /**
     * 检查某个位置是否被占用
     */
    public boolean isOccupied(float x, float y) {
        if (!isInitialized || gridMapBuilder == null) {
            return false;
        }
        return gridMapBuilder.isOccupied(x, y);
    }

    /**
     * 设置自动保存
     */
    public void setAutoSave(boolean enabled) {
        this.autoSaveEnabled = enabled;
        Log.i(TAG, "自动保存设置为: " + enabled);
    }

    /**
     * 获取自动保存状态
     */
    public boolean isAutoSaveEnabled() {
        return autoSaveEnabled;
    }

    /**
     * 检查ROS订阅是否活跃
     */
    public boolean isRosSubscriptionActive() {
        return isRosSubscriptionActive.get();
    }

    /**
     * 显示Toast消息
     */
    private void showToast(String message) {
        if (activity != null) {
            mainHandler.post(() ->
                    Toast.makeText(activity, message, Toast.LENGTH_SHORT).show());
        }
    }

    /**
     * 关闭并清理资源
     */
    public void shutdown() {
        try {
            // 最后保存一次地图
            if (isInitialized && autoSaveEnabled) {
                saveGridMapInternal();
            }

            // 清理资源
            isRosSubscriptionActive.set(false);

            if (ptsAndPoseSubscriber != null) {
                ptsAndPoseSubscriber.shutdown();
                ptsAndPoseSubscriber = null;
            }

            if (allKfAndPtsSubscriber != null) {
                allKfAndPtsSubscriber.shutdown();
                allKfAndPtsSubscriber = null;
            }

            connectedNode = null;
            gridMapBuilder = null;
            isInitialized = false;

            // 关闭线程池
            if (backgroundExecutor != null && !backgroundExecutor.isShutdown()) {
                backgroundExecutor.shutdown();
            }

            Log.i(TAG, "网格地图管理器已关闭");

        } catch (Exception e) {
            Log.e(TAG, "关闭网格地图管理器时出错: " + e.getMessage(), e);
        }
    }

    // ===== NodeMain接口实现 =====

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("android/grid_map_manager");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;

        try {
            // 订阅关键帧数据话题
            ptsAndPoseSubscriber = connectedNode.newSubscriber("/pts_and_pose", PoseArray._TYPE);
            ptsAndPoseSubscriber.addMessageListener(message -> {
                try {
                    if (message != null && message.getPoses().size() > 1) {
                        // 转换ROS消息为float数组
                        float[] keyFrameData = convertPoseArrayToKeyFrameData(message);
                        if (keyFrameData != null) {
                            processKeyframeData(keyFrameData);
                        }
                    }
                } catch (Exception e) {
                    Log.e(TAG, "处理/pts_and_pose消息时出错: " + e.getMessage(), e);
                }
            });

            // 订阅所有关键帧数据话题（闭环重建）
            allKfAndPtsSubscriber = connectedNode.newSubscriber("/all_kf_and_pts", PoseArray._TYPE);
            allKfAndPtsSubscriber.addMessageListener(message -> {
                try {
                    if (message != null && message.getPoses().size() > 1) {
                        // 转换ROS消息为float数组
                        float[] allKeyframesData = convertPoseArrayToAllKeyframesData(message);
                        if (allKeyframesData != null) {
                            processAllKeyframesData(allKeyframesData);
                        }
                    }
                } catch (Exception e) {
                    Log.e(TAG, "处理/all_kf_and_pts消息时出错: " + e.getMessage(), e);
                }
            });

            isRosSubscriptionActive.set(true);
            Log.i(TAG, "网格地图ROS订阅已启动");

        } catch (Exception e) {
            Log.e(TAG, "启动网格地图ROS订阅时出错: " + e.getMessage(), e);
        }
    }

    @Override
    public void onShutdown(Node node) {
        isRosSubscriptionActive.set(false);
        Log.i(TAG, "网格地图ROS节点已关闭");
    }

    @Override
    public void onShutdownComplete(Node node) {
        Log.i(TAG, "网格地图ROS节点完全关闭");
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        Log.e(TAG, "网格地图ROS节点错误: " + throwable.getMessage(), throwable);
        isRosSubscriptionActive.set(false);
    }

    // ===== ROS消息转换方法 =====

    /**
     * 转换PoseArray消息为关键帧数据
     */
    private float[] convertPoseArrayToKeyFrameData(PoseArray message) {
        try {
            if (message.getPoses().size() < 1) return null;

            // 第一个pose是关键帧位姿，后续poses是地图点
            geometry_msgs.Pose kfPose = message.getPoses().get(0);
            int numMapPoints = message.getPoses().size() - 1;

            float[] keyFrameData = new float[7 + numMapPoints * 3];

            // 关键帧位姿（前7个元素）
            keyFrameData[0] = (float) kfPose.getPosition().getX();
            keyFrameData[1] = (float) kfPose.getPosition().getY();
            keyFrameData[2] = (float) kfPose.getPosition().getZ();
            keyFrameData[3] = (float) kfPose.getOrientation().getX();
            keyFrameData[4] = (float) kfPose.getOrientation().getY();
            keyFrameData[5] = (float) kfPose.getOrientation().getZ();
            keyFrameData[6] = (float) kfPose.getOrientation().getW();

            // 地图点（每3个元素一个点）
            for (int i = 0; i < numMapPoints; i++) {
                geometry_msgs.Pose pointPose = message.getPoses().get(i + 1);
                int baseIndex = 7 + i * 3;
                keyFrameData[baseIndex] = (float) pointPose.getPosition().getX();
                keyFrameData[baseIndex + 1] = (float) pointPose.getPosition().getY();
                keyFrameData[baseIndex + 2] = (float) pointPose.getPosition().getZ();
            }

            return keyFrameData;

        } catch (Exception e) {
            Log.e(TAG, "转换关键帧数据时出错: " + e.getMessage(), e);
            return null;
        }
    }

    /**
     * 转换PoseArray消息为所有关键帧数据
     */
    private float[] convertPoseArrayToAllKeyframesData(PoseArray message) {
        try {
            if (message.getPoses().size() < 3) return null;

            // 第一个pose包含关键帧数量
            int nKf = (int) message.getPoses().get(0).getPosition().getX();

            // 预估数据大小并创建数组
            float[] allKeyframesData = new float[message.getPoses().size() * 8]; // 预估大小
            int writeIndex = 0;

            // 写入关键帧数量
            allKeyframesData[writeIndex++] = nKf;

            int readIndex = 1;
            for (int i = 0; i < nKf && readIndex < message.getPoses().size(); i++) {
                // 关键帧位姿
                geometry_msgs.Pose kfPose = message.getPoses().get(readIndex++);
                allKeyframesData[writeIndex++] = (float) kfPose.getPosition().getX();
                allKeyframesData[writeIndex++] = (float) kfPose.getPosition().getY();
                allKeyframesData[writeIndex++] = (float) kfPose.getPosition().getZ();
                allKeyframesData[writeIndex++] = (float) kfPose.getOrientation().getX();
                allKeyframesData[writeIndex++] = (float) kfPose.getOrientation().getY();
                allKeyframesData[writeIndex++] = (float) kfPose.getOrientation().getZ();
                allKeyframesData[writeIndex++] = (float) kfPose.getOrientation().getW();

                // 地图点数量
                if (readIndex >= message.getPoses().size()) break;
                geometry_msgs.Pose ptsCountPose = message.getPoses().get(readIndex++);
                int nPts = (int) ptsCountPose.getPosition().getX();
                allKeyframesData[writeIndex++] = nPts;

                // 地图点
                for (int j = 0; j < nPts && readIndex < message.getPoses().size(); j++) {
                    geometry_msgs.Pose pointPose = message.getPoses().get(readIndex++);
                    allKeyframesData[writeIndex++] = (float) pointPose.getPosition().getX();
                    allKeyframesData[writeIndex++] = (float) pointPose.getPosition().getY();
                    allKeyframesData[writeIndex++] = (float) pointPose.getPosition().getZ();
                }
            }

            // 创建正确大小的数组
            float[] result = new float[writeIndex];
            System.arraycopy(allKeyframesData, 0, result, 0, writeIndex);

            return result;

        } catch (Exception e) {
            Log.e(TAG, "转换所有关键帧数据时出错: " + e.getMessage(), e);
            return null;
        }
    }
}