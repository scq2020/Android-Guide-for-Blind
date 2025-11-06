// PathPlanningManager.java
package com.vslam.orbslam3.vslamactivity;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * 路径规划管理器 - 集成路径优化功能
 * 功能：在地图上规划路径并显示，支持多种路径优化算法
 */
public class PathPlanningManager {
    private static final String TAG = "PathPlanningManager";

    // 路径规划参数
    private static final double DEFAULT_WEIGHT_A = 1.0;  // A*算法启发式权重
    private static final double DEFAULT_WEIGHT_B = 1.0;  // A*算法启发式权重
    private static final String DEFAULT_DISTANCE_TYPE = "euclidean"; // 距离类型

    // 路径显示样式
    private static final int PATH_COLOR = Color.RED;           // 路径颜色
    private static final int OPTIMIZED_PATH_COLOR = Color.BLUE; // 优化路径颜色
    private static final int START_POINT_COLOR = Color.GREEN;  // 起点颜色
    private static final int END_POINT_COLOR = Color.BLUE;     // 终点颜色
    private static final int TURNING_POINT_COLOR = Color.MAGENTA; // 转折点颜色
    private static final float PATH_STROKE_WIDTH = 3.0f;      // 路径线宽
    private static final float OPTIMIZED_PATH_STROKE_WIDTH = 1.0f; // 优化路径线宽
    private static final float POINT_RADIUS = 2.0f;           // 点的半径

    // 组件引用
    private final Context context;
    private final MapServerManager mapServerManager;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final ExecutorService executor = Executors.newSingleThreadExecutor();

    // 路径数据
    private List<PathPoint> originalPath;          // 原始A*路径
    private List<PathPoint> currentPath;           // 当前优化后路径点列表
    private List<PathPoint> turningPoints;         // 转折点列表
    private PathPoint startPoint;                  // 起点
    private PathPoint endPoint;                    // 终点
    private boolean pathPlanningInProgress = false; // 路径规划进行中标志

    // 路径规划配置
    private PathPlanningConfig planningConfig;
    private float[] lastCameraPose; // 相机的四元数朝向
    private double[] lastPlanningPosition; // 路径起点世界坐标
    private DebugLogger debugLogger;

    private PathPoint snappedEndPoint; // 新增：实际用于路径规划和导航的吸附终点
    private static final int SNAPPED_POINT_COLOR = Color.YELLOW; // 吸附点特显色

    private static final int ORIGINAL_TARGET_COLOR = Color.CYAN; // 原始目标点颜色
    /**
     * 路径点数据结构
     */
    public static class PathPoint {
        public double worldX;    // 世界坐标X (米)
        public double worldY;    // 世界坐标Y (米)
        public int gridX;        // 栅格坐标X
        public int gridY;        // 栅格坐标Y

        public PathPoint(double worldX, double worldY) {
            this.worldX = worldX;
            this.worldY = worldY;
        }

        public PathPoint(int gridX, int gridY) {
            this.gridX = gridX;
            this.gridY = gridY;
        }

        public PathPoint(PathPoint other) {
            this.worldX = other.worldX;
            this.worldY = other.worldY;
            this.gridX = other.gridX;
            this.gridY = other.gridY;
        }

        @Override
        public String toString() {
            return String.format("World(%.3f,%.3f) Grid(%d,%d)", worldX, worldY, gridX, gridY);
        }
    }

    /**
     * 路径规划配置类 - 扩展优化功能
     */
    public static class PathPlanningConfig {
        // A*算法参数
        public double weightA = DEFAULT_WEIGHT_A;
        public double weightB = DEFAULT_WEIGHT_B;
        public String distanceType = DEFAULT_DISTANCE_TYPE;

        // 显示配置
        public int pathColor = PATH_COLOR;
        public int optimizedPathColor = OPTIMIZED_PATH_COLOR;
        public int startPointColor = START_POINT_COLOR;
        public int endPointColor = END_POINT_COLOR;
        public int turningPointColor = TURNING_POINT_COLOR;
        public float pathStrokeWidth = PATH_STROKE_WIDTH;
        public float optimizedPathStrokeWidth = OPTIMIZED_PATH_STROKE_WIDTH;
        public float pointRadius = POINT_RADIUS;

        // 路径优化配置
        public boolean enablePathOptimization = true;      // 启用路径优化
        public boolean enablePathSmoothing = true;         // 启用路径平滑
        public boolean useDouglasPeucker = true;           // 使用Douglas-Peucker算法
        public boolean useLineDetection = true;            // 使用直线检测优化
        public boolean showOriginalPath = false;           // 显示原始路径（调试用）
        public boolean showTurningPoints = true;           // 显示转折点

        // 优化参数
        public double smoothingTolerance = 0.05;           // 平滑处理阈值（米）
        public double douglasPeuckerTolerance = 0.03;      // DP算法阈值（米）
        public int maxLineDetectionDistance = 20;          // 直线检测最大距离（栅格）
        public double minSegmentLength = 0.1;             // 最小路径段长度（米）
    }

    /**
     * 路径规划回调接口
     */
    public interface PathPlanningCallback {
        void onPathPlanningStarted();
        void onPathPlanningCompleted(List<PathPoint> path);
        void onPathPlanningFailed(String error);
        default void onPathOptimizationCompleted(List<PathPoint> originalPath, List<PathPoint> optimizedPath) {}
    }

    /**
     * 构造函数
     */
    public PathPlanningManager(Context context, MapServerManager mapServerManager) {
        this.context = context;
        this.mapServerManager = mapServerManager;
        this.planningConfig = new PathPlanningConfig();
        this.originalPath = new ArrayList<>();
        this.currentPath = new ArrayList<>();
        this.turningPoints = new ArrayList<>();
        this.debugLogger = new DebugLogger(context);

        Log.i(TAG, "PathPlanningManager初始化完成");
    }

    /**
     * 设置路径规划配置
     */
    public void setPathPlanningConfig(PathPlanningConfig config) {
        this.planningConfig = config;
        Log.i(TAG, "路径规划配置已更新");
    }


    /**
     * 规划指定起点和终点的路径
     * @param startX 起点X坐标 (米)
     * @param startY 起点Y坐标 (米)
     * @param endX 终点X坐标 (米)
     * @param endY 终点Y坐标 (米)
     * @param callback 规划完成回调
     */
    public void planPath(double startX, double startY, double endX, double endY,
                         PathPlanningCallback callback) {
        //debugLogger.updateDebugTextView("❌ ================================" );//调试信息的例子
        if (pathPlanningInProgress) {
            Log.w(TAG, "路径规划正在进行中，请稍后重试");
            if (callback != null) {
                callback.onPathPlanningFailed("路径规划正在进行中");
            }
            return;
        }

        if (!mapServerManager.isMapLoaded()) {
            Log.e(TAG, "地图未加载，无法进行路径规划");
            if (callback != null) {
                callback.onPathPlanningFailed("地图未加载");
            }
            return;
        }

        pathPlanningInProgress = true;

        // 创建起点和终点
        startPoint = new PathPoint(startX, startY);
        endPoint = new PathPoint(endX, endY);

        if (callback != null) {
            callback.onPathPlanningStarted();
        }

        executor.execute(() -> {
            try {
                // 执行路径规划
                List<PathPoint> path = performPathPlanning(startPoint, endPoint);

                pathPlanningInProgress = false;

                if (path != null && !path.isEmpty()) {
                    currentPath = path;

                    // 在主线程更新界面
                    mainHandler.post(() -> {
                        drawPathOnMap();
                        if (callback != null) {
                            callback.onPathPlanningCompleted(path);
                            if (planningConfig.enablePathOptimization) {
                                callback.onPathOptimizationCompleted(originalPath, currentPath);
                            }
                        }
                        showToast("路径规划成功，共" + path.size() + "个路径点");
                    });

                    Log.i(TAG, "路径规划成功，路径长度: " + path.size() + " 点");
                } else {
                    mainHandler.post(() -> {
                        if (callback != null) {
                            callback.onPathPlanningFailed("未找到有效路径");
                        }
                        showToast("路径规划失败：未找到有效路径");
                    });

                    Log.e(TAG, "路径规划失败：未找到有效路径");
                }

            } catch (Exception e) {
                pathPlanningInProgress = false;
                Log.e(TAG, "路径规划异常: " + e.getMessage(), e);

                mainHandler.post(() -> {
                    if (callback != null) {
                        callback.onPathPlanningFailed("路径规划异常: " + e.getMessage());
                    }
                    showToast("路径规划失败: " + e.getMessage());
                });
            }
        });
    }

    /**
     * 执行实际的路径规划计算
     */
    private List<PathPoint> performPathPlanning(PathPoint start, PathPoint end) {
        try {




            // 每次都重新置空snappedEndPoint
            snappedEndPoint = null;
            // 1. 获取地图信息和数据
            MapServerManager.MapInfo mapInfo = mapServerManager.getMapInfo();
            byte[] mapData = mapServerManager.getMapDataCopy();
            if (mapInfo == null || mapData == null) {
                throw new Exception("无法获取地图数据");
            }



            // 2. 世界坐标 → 栅格坐标
            start.gridX = worldToGridX(start.worldX, mapInfo);
            start.gridY = worldToGridY(start.worldY, mapInfo);
            end.gridX   = worldToGridX(end.worldX,   mapInfo);
            end.gridY   = worldToGridY(end.worldY,   mapInfo);

            // 3. 检查起点、终点是否可通行，必要时用 BFS 找最近可通行点
            if (!isGridPointNavigable(start.gridX, start.gridY, mapData, mapInfo)) {
                Log.i(TAG, "起点不可通行，开始搜索最近可通行起点");
                start = processGoalPointIfBlocked(start, mapData, mapInfo);
                if (start == null) throw new Exception("起点周围无可通行点");
            }

            if (!isGridPointNavigable(end.gridX, end.gridY, mapData, mapInfo)) {
                Log.i(TAG, "终点不可通行，开始搜索最近可通行终点");
                snappedEndPoint = processGoalPointIfBlocked(end, mapData, mapInfo);
                if (snappedEndPoint == null) throw new Exception("终点周围无可通行点");
            } else {
                // 如果终点本就可通行
                snappedEndPoint = new PathPoint(end);
            }

            // 4. 转换地图数据为 A* 格式
            int[] astarMapData = convertMapDataForAstar(mapData);


            // 5. 调用 native A*
            int[] pathResult = nativeFindPath(
                    astarMapData,
                    mapInfo.width,
                    mapInfo.height,
                    start.gridX, start.gridY,
                    snappedEndPoint.gridX, snappedEndPoint.gridY, // 用吸附点而不是原始end
                    planningConfig.weightA,
                    planningConfig.weightB,
                    planningConfig.distanceType
            );

            // 7. 处理路径结果
            if (pathResult == null || pathResult.length == 0) {
                return null;
            }


            // 8. 将栅格路径转换为路径点列表
            List<PathPoint> pathPoints = new ArrayList<>();
            for (int i = 0; i < pathResult.length; i += 2) {
                int gridX = pathResult[i];
                int gridY = pathResult[i + 1];

                PathPoint point = new PathPoint(gridX, gridY);
                point.worldX = gridToWorldX(gridX, mapInfo);
                point.worldY = gridToWorldY(gridY, mapInfo);

                pathPoints.add(point);
            }

            // 保存原始路径
            originalPath = new ArrayList<>();
            for (PathPoint point : pathPoints) {
                originalPath.add(new PathPoint(point));
            }

            Log.i(TAG, String.format("A*算法返回 %d 个原始路径点", pathPoints.size()));

            // 9. 路径优化处理
            if (planningConfig.enablePathOptimization && pathPoints.size() > 2) {
                Log.i(TAG, "开始路径优化处理");
                pathPoints = optimizePath(pathPoints, mapInfo, mapData);
                Log.i(TAG, String.format("路径优化完成，优化后点数: %d", pathPoints.size()));
            } else if (planningConfig.enablePathSmoothing && pathPoints.size() > 2) {
                // 仅简单平滑
                pathPoints = smoothPath(pathPoints);
            }



            return pathPoints;

        } catch (Exception e) {
            Log.e(TAG, "路径规划计算失败: " + e.getMessage(), e);
            throw new RuntimeException("路径规划计算失败", e);
        }
    }

    /**
     * 路径优化主入口 - 集成多种优化算法
     */
    private List<PathPoint> optimizePath(List<PathPoint> originalPath,
                                         MapServerManager.MapInfo mapInfo,
                                         byte[] mapData) {
        List<PathPoint> optimizedPath = new ArrayList<>(originalPath);

        try {
            Log.i(TAG, "原始路径点数: " + optimizedPath.size());

            // 1. Douglas-Peucker算法简化
            if (planningConfig.useDouglasPeucker && optimizedPath.size() > 2) {
                optimizedPath = simplifyPathDouglasPeucker(optimizedPath,
                        planningConfig.douglasPeuckerTolerance);
                Log.i(TAG, "Douglas-Peucker简化后: " + optimizedPath.size() + " 点");
            }

            // 2. 直线检测优化
            if (planningConfig.useLineDetection && optimizedPath.size() > 2) {
                optimizedPath = optimizeWithLineDetection(optimizedPath, mapInfo, mapData);
                Log.i(TAG, "直线检测优化后: " + optimizedPath.size() + " 点");
            }

            // 3. 检测转折点
            turningPoints = findTurningPoints(optimizedPath);
            Log.i(TAG, "检测到 " + turningPoints.size() + " 个转折点");

            // 4. 最终平滑处理
            if (planningConfig.enablePathSmoothing && optimizedPath.size() > 2) {
                optimizedPath = smoothPath(optimizedPath);
                Log.i(TAG, "最终平滑后: " + optimizedPath.size() + " 点");
            }

            // 5. 移除过短的路径段
            optimizedPath = removeShortSegments(optimizedPath, planningConfig.minSegmentLength);
            Log.i(TAG, "移除短段后: " + optimizedPath.size() + " 点");

        } catch (Exception e) {
            Log.e(TAG, "路径优化过程异常: " + e.getMessage(), e);
            return originalPath; // 失败时返回原始路径
        }

        return optimizedPath;
    }

    /**
     * Douglas-Peucker算法简化路径
     */
    private List<PathPoint> simplifyPathDouglasPeucker(List<PathPoint> path, double tolerance) {
        if (path.size() <= 2) {
            return path;
        }

        List<PathPoint> simplified = new ArrayList<>();
        simplified.add(path.get(0)); // 起点

        // 递归简化
        douglasPeuckerRecursive(path, 0, path.size() - 1, tolerance, simplified);

        simplified.add(path.get(path.size() - 1)); // 终点

        return simplified;
    }

    /**
     * 获取当前路径的总长度
     */
    public double getCurrentPathLength() {
        return calculatePathLength(currentPath);
    }


    private void douglasPeuckerRecursive(List<PathPoint> path, int start, int end,
                                         double tolerance, List<PathPoint> result) {
        if (end - start <= 1) {
            return;
        }

        // 找到距离直线最远的点
        double maxDistance = 0;
        int maxIndex = -1;

        PathPoint startPoint = path.get(start);
        PathPoint endPoint = path.get(end);

        for (int i = start + 1; i < end; i++) {
            PathPoint current = path.get(i);
            double distance = calculatePointToLineDistance(current, startPoint, endPoint);

            if (distance > maxDistance) {
                maxDistance = distance;
                maxIndex = i;
            }
        }

        // 如果最大距离超过阈值，递归处理
        if (maxDistance > tolerance && maxIndex != -1) {
            douglasPeuckerRecursive(path, start, maxIndex, tolerance, result);
            result.add(path.get(maxIndex));
            douglasPeuckerRecursive(path, maxIndex, end, tolerance, result);
        }
    }


    /**
     * ✅ 修复：计算相机朝向与路径方向之间的夹角 - 统一语义
     * @param path 当前路径
     * @param cameraPose 相机姿态 [x, y, z, qx, qy, qz, qw]
     * @return 夹角（单位：度，左转为负，右转为正）
     */
    public float calculatePathHeadingAngle(List<PathPoint> path, float[] cameraPose) {
        if (path == null || path.size() < 2 || cameraPose == null || cameraPose.length < 7) {
            return 0.0f;
        }

        // 1. 路径方向向量（第一段）
        PathPoint realStart = path.get(path.size() - 1);
        PathPoint realNext = path.get(path.size() - 2);

        double dxPath = realNext.worldX - realStart.worldX;
        double dyPath = realNext.worldY - realStart.worldY;

        double pathAngle = Math.atan2(dyPath, dxPath);

        // 2. 相机朝向角度（已修改为正确的坐标系）
        double cameraYaw = getYawFromQuaternion(cameraPose[3], cameraPose[4], cameraPose[5], cameraPose[6]);

        // 3. 计算角度差
        double angleDiff = pathAngle - cameraYaw;

        // 4. 规范化到 [-π, π]
        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
        while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

        return (float) -Math.toDegrees(angleDiff);
    }
    /**
     * 直线检测优化 - 类似ROS的实现
     */
    private List<PathPoint> optimizeWithLineDetection(List<PathPoint> originalPath,
                                                      MapServerManager.MapInfo mapInfo,
                                                      byte[] mapData) {
        List<PathPoint> optimized = new ArrayList<>();

        if (originalPath.isEmpty()) {
            return optimized;
        }

        optimized.add(originalPath.get(0)); // 添加起点

        int currentIndex = 0;
        while (currentIndex < originalPath.size() - 1) {
            int farthestReachable = currentIndex;

            // 从当前点开始，找最远可直接到达的点
            for (int i = currentIndex + 1; i < originalPath.size() &&
                    i <= currentIndex + planningConfig.maxLineDetectionDistance; i++) {

                if (canReachDirectly(originalPath.get(currentIndex),
                        originalPath.get(i), mapInfo, mapData)) {
                    farthestReachable = i;
                } else {
                    break; // 遇到障碍物就停止
                }
            }

            // 如果能跳跃多个点，就直接到最远点
            if (farthestReachable > currentIndex + 1) {
                currentIndex = farthestReachable;
                optimized.add(originalPath.get(currentIndex));
            } else {
                // 否则只前进一步
                currentIndex++;
                optimized.add(originalPath.get(currentIndex));
            }
        }

        return optimized;
    }

    /**
     * 检查两点间是否可直接到达（无障碍物）
     */
    private boolean canReachDirectly(PathPoint start, PathPoint end,
                                     MapServerManager.MapInfo mapInfo, byte[] mapData) {
        try {
            // Bresenham直线算法检查路径上所有点
            List<int[]> linePoints = getLinePoints(
                    worldToGridX(start.worldX, mapInfo),
                    worldToGridY(start.worldY, mapInfo),
                    worldToGridX(end.worldX, mapInfo),
                    worldToGridY(end.worldY, mapInfo)
            );

            // 检查直线上每个点是否可通行
            for (int[] point : linePoints) {
                if (!isGridPointNavigable(point[0], point[1], mapData, mapInfo)) {
                    return false;
                }
            }

            return true;
        } catch (Exception e) {
            Log.e(TAG, "检查直线可达性失败: " + e.getMessage());
            return false;
        }
    }

    /**
     * Bresenham直线算法获取直线上所有点
     */
    private List<int[]> getLinePoints(int x0, int y0, int x1, int y1) {
        List<int[]> points = new ArrayList<>();

        int dx = Math.abs(x1 - x0);
        int dy = Math.abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2;
        int e2;

        while (true) {
            points.add(new int[]{x0, y0});

            if (x0 == x1 && y0 == y1) break;

            e2 = err;
            if (e2 > -dx) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dy) {
                err += dx;
                y0 += sy;
            }
        }

        return points;
    }

    /**
     * 寻找转折点
     */
    private List<PathPoint> findTurningPoints(List<PathPoint> path) {
        List<PathPoint> turningPoints = new ArrayList<>();

        if (path.size() < 3) {
            return turningPoints;
        }

        for (int i = 1; i < path.size() - 1; i++) {
            PathPoint prev = path.get(i - 1);
            PathPoint curr = path.get(i);
            PathPoint next = path.get(i + 1);

            // 计算方向变化角度
            double angle = calculateTurningAngle(prev, curr, next);

            // 如果角度变化超过阈值，认为是转折点
            if (angle > 15.0) { // 15度阈值
                turningPoints.add(curr);
            }
        }

        return turningPoints;
    }

    /**
     * 计算转折角度
     */
    private double calculateTurningAngle(PathPoint prev, PathPoint curr, PathPoint next) {
        // 计算两个向量
        double v1x = curr.worldX - prev.worldX;
        double v1y = curr.worldY - prev.worldY;
        double v2x = next.worldX - curr.worldX;
        double v2y = next.worldY - curr.worldY;

        // 计算向量模长
        double mag1 = Math.sqrt(v1x * v1x + v1y * v1y);
        double mag2 = Math.sqrt(v2x * v2x + v2y * v2y);

        if (mag1 == 0 || mag2 == 0) {
            return 0;
        }

        // 计算夹角
        double dot = v1x * v2x + v1y * v2y;
        double cosAngle = dot / (mag1 * mag2);

        // 防止浮点误差
        cosAngle = Math.max(-1.0, Math.min(1.0, cosAngle));

        // 转换为角度
        return Math.toDegrees(Math.acos(cosAngle));
    }

    /**
     * 移除过短的路径段
     */
    private List<PathPoint> removeShortSegments(List<PathPoint> path, double minLength) {
        if (path.size() <= 2) {
            return path;
        }

        List<PathPoint> filtered = new ArrayList<>();
        filtered.add(path.get(0)); // 起点

        for (int i = 1; i < path.size() - 1; i++) {
            PathPoint prev = filtered.get(filtered.size() - 1);
            PathPoint curr = path.get(i);

            double distance = Math.sqrt(
                    Math.pow(curr.worldX - prev.worldX, 2) +
                            Math.pow(curr.worldY - prev.worldY, 2)
            );

            if (distance >= minLength) {
                filtered.add(curr);
            }
        }

        filtered.add(path.get(path.size() - 1)); // 终点
        return filtered;
    }

    /**
     * 路径平滑处理（保留原有方法）
     */
    private List<PathPoint> smoothPath(List<PathPoint> originalPath) {
        List<PathPoint> smoothedPath = new ArrayList<>();

        if (originalPath.size() <= 2) {
            return originalPath;
        }

        smoothedPath.add(originalPath.get(0)); // 添加起点

        for (int i = 1; i < originalPath.size() - 1; i++) {
            PathPoint prev = originalPath.get(i - 1);
            PathPoint curr = originalPath.get(i);
            PathPoint next = originalPath.get(i + 1);

            // 计算偏离直线的距离
            double deviation = calculatePointToLineDistance(curr, prev, next);

            // 如果偏离距离大于阈值，保留该点
            if (deviation > planningConfig.smoothingTolerance) {
                smoothedPath.add(curr);
            }
        }

        smoothedPath.add(originalPath.get(originalPath.size() - 1)); // 添加终点

        return smoothedPath;
    }
    public void setCameraPose(float[] cameraPose) {
        this.lastCameraPose = cameraPose;
    }
    /**
     * 在地图上绘制路径（扩展版 - 支持多种路径显示）
     */
    private void drawPathOnMap() {
        // 检查ImageView是否已设置
        if (!mapServerManager.isImageViewSet()) {
            Log.w(TAG, "MapImageView未设置，无法绘制路径");
            return;
        }

        // 获取地图显示位图（原始尺寸）
        Bitmap originalBitmap = mapServerManager.getMapDisplayBitmap();
        if (originalBitmap == null) {
            Log.w(TAG, "无法获取地图位图");
            return;
        }

        // 创建可修改的位图副本
        Bitmap mutableBitmap = originalBitmap.copy(Bitmap.Config.ARGB_8888, true);
        Canvas canvas = new Canvas(mutableBitmap);

        // 绘制原始路径（如果启用）planningConfig.showOriginalPath &&
        if (originalPath.size() > 1) {
            drawPath(canvas, originalPath, planningConfig.pathColor,
                    planningConfig.pathStrokeWidth, true);
        }

        // 绘制优化后的路径
        if (currentPath.size() > 1) {
            drawPath(canvas, currentPath, planningConfig.optimizedPathColor,
                    planningConfig.optimizedPathStrokeWidth, false);
        }

// 绘制起点
        if (startPoint != null) {
            drawPoint(canvas, startPoint, planningConfig.startPointColor, planningConfig.pointRadius);
        }

// 绘制终点 - 统一逻辑，不重复
        if (snappedEndPoint != null) {
            // 如果有吸附点，绘制吸附点
            drawPoint(canvas, snappedEndPoint, SNAPPED_POINT_COLOR, planningConfig.pointRadius * 1.5f);
            Log.i(TAG, "绘制吸附点: " + snappedEndPoint.toString());
        } else if (endPoint != null) {
            // 如果没有吸附点，绘制原终点
            drawPoint(canvas, endPoint, planningConfig.endPointColor, planningConfig.pointRadius);
            Log.i(TAG, "绘制原终点: " + endPoint.toString());
        }
        // 绘制原始目标点（如果有吸附点的话）
        if (snappedEndPoint != null && endPoint != null) {
            drawPoint(canvas, endPoint, ORIGINAL_TARGET_COLOR, planningConfig.pointRadius);
            Log.i(TAG, "绘制原始目标点: " + endPoint.toString());
        }
        // 绘制转折点
        if (planningConfig.showTurningPoints && turningPoints != null) {
            for (PathPoint turningPoint : turningPoints) {
                drawPoint(canvas, turningPoint, planningConfig.turningPointColor,
                        planningConfig.pointRadius);
            }
        }

        // ✅ 更新显示 - 位图会在MapServerManager中被正确缩放显示
        mapServerManager.updateMapDisplayBitmap(mutableBitmap);
    }
    public void redrawPathAndArrow(Canvas canvas, double[] robotPos, float[] cameraPose) {
        // 1. 重画路径
        if (currentPath.size() > 1) {
            drawPath(canvas, currentPath, planningConfig.optimizedPathColor,
                    planningConfig.optimizedPathStrokeWidth, false);
        }

        // 2. 重画起点
        if (startPoint != null) {
            drawPoint(canvas, startPoint, planningConfig.startPointColor, planningConfig.pointRadius);
        }

        // ✅ 修复：优先绘制吸附点，而不是原始终点
        if (snappedEndPoint != null) {
            drawPoint(canvas, snappedEndPoint, SNAPPED_POINT_COLOR, planningConfig.pointRadius * 1.5f);
        } else if (endPoint != null) {
            drawPoint(canvas, endPoint, planningConfig.endPointColor, planningConfig.pointRadius);
        }
        // 绘制原始目标点（如果有吸附点的话）
        if (snappedEndPoint != null && endPoint != null) {
            drawPoint(canvas, endPoint, ORIGINAL_TARGET_COLOR, planningConfig.pointRadius);
        }
        // 3. 重画转折点
        if (planningConfig.showTurningPoints && turningPoints != null) {
            for (PathPoint tp : turningPoints) {
                drawPoint(canvas, tp, planningConfig.turningPointColor, planningConfig.pointRadius);
            }
        }

        // 4. 重画箭头
        drawCameraOrientation(canvas, robotPos, cameraPose);
    }
    public void updateRobotPose(float[] cameraPose) {
        this.lastCameraPose = cameraPose;
        this.lastPlanningPosition = new double[]{
                cameraPose[0],
                cameraPose[1],
                cameraPose[2]
        };
    }
    public void drawCameraOrientation(Canvas canvas,
                                       double[] startPosition,
                                       float[] cameraPose) {

        if (cameraPose == null || cameraPose.length < 7) return;

        // 1. 图像坐标转换
        float[] coords = mapServerManager.mapToImageCoordinates(
                startPosition[0], startPosition[1]);
        if (coords == null) return;

        float x = coords[0];
        float y = coords[1];   // 向下偏移 10 像素，避免被起点圆点遮住

        // 2. 计算方向向量
        float yaw = getYawFromQuaternion(
                cameraPose[3], cameraPose[4], cameraPose[5], cameraPose[6]);
        float len = 20f;
        float dx = (float) Math.cos(yaw) * len;
        float dy = (float) Math.sin(yaw) * len;

        float endX = x + dx;
        float endY = y - dy;        // 图像 Y 轴向下为正，需取反

        // 3. 绘制主箭头线段
        Paint linePaint = new Paint();
        linePaint.setColor(Color.RED);
        linePaint.setStrokeWidth(3f);
        linePaint.setAntiAlias(true);
        linePaint.setStyle(Paint.Style.STROKE);
        canvas.drawLine(x, y, endX, endY, linePaint);

        // 4. 绘制小箭头三角头
        float arrowSize = 8f;               // 三角边长
        float angle = (float) Math.toRadians(30); // 夹角 30°

        float arrowX1 = endX - arrowSize * (float) Math.cos(yaw - angle);
        float arrowY1 = endY + arrowSize * (float) Math.sin(yaw - angle);

        float arrowX2 = endX - arrowSize * (float) Math.cos(yaw + angle);
        float arrowY2 = endY + arrowSize * (float) Math.sin(yaw + angle);

        Paint headPaint = new Paint(linePaint);
        headPaint.setStyle(Paint.Style.FILL);

        Path arrowHead = new Path();
        arrowHead.moveTo(endX, endY);
        arrowHead.lineTo(arrowX1, arrowY1);
        arrowHead.lineTo(arrowX2, arrowY2);
        arrowHead.close();

        canvas.drawPath(arrowHead, headPaint);
    }

    private float getYawFromQuaternion(float qx, float qy, float qz, float qw) {
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        return (float) Math.atan2(siny_cosp, cosy_cosp);
    }
    /**
     * 绘制单条路径
     */
    private void drawPath(Canvas canvas, List<PathPoint> path, int color,
                          float strokeWidth, boolean isDashed) {
        if (path.size() < 2) return;

        Paint pathPaint = new Paint();
        pathPaint.setColor(color);
        pathPaint.setStrokeWidth(strokeWidth);
        pathPaint.setStyle(Paint.Style.STROKE);
        pathPaint.setAntiAlias(true);

        if (isDashed) {
            pathPaint.setPathEffect(new android.graphics.DashPathEffect(new float[]{10, 5}, 0));
        }

        Path pathGeometry = new Path();
        boolean firstPoint = true;

        for (PathPoint point : path) {
            // ✅ 使用图像坐标转换（不缩放）
            float[] coords = mapServerManager.mapToImageCoordinates(
                    point.worldX, point.worldY);

            if (coords != null) {
                if (firstPoint) {
                    pathGeometry.moveTo(coords[0], coords[1]);
                    firstPoint = false;
                } else {
                    pathGeometry.lineTo(coords[0], coords[1]);
                }
            }
        }

        canvas.drawPath(pathGeometry, pathPaint);
    }

    /**
     * 绘制单个点
     */
    private void drawPoint(Canvas canvas, PathPoint point, int color, float radius) {
        Paint pointPaint = new Paint();
        pointPaint.setColor(color);
        pointPaint.setAntiAlias(true);
        pointPaint.setStyle(Paint.Style.FILL);

        // ✅ 使用图像坐标转换（不缩放）
        float[] coords = mapServerManager.mapToImageCoordinates(
                point.worldX, point.worldY);

        if (coords != null) {
            canvas.drawCircle(coords[0], coords[1], radius, pointPaint);
        }
    }

    /**
     * 清除当前路径显示
     */
    public void clearPath() {
        originalPath.clear();
        currentPath.clear();
        turningPoints.clear();
        startPoint = null;
        endPoint = null;

        // 刷新地图显示（移除路径）
        mapServerManager.refreshMapDisplay();

        Log.i(TAG, "路径已清除");
    }

    /**
     * ✅ 统一坐标系：世界坐标X -> 栅格坐标X
     */
    private int worldToGridX(double worldX, MapServerManager.MapInfo mapInfo) {
        return (int) Math.round((worldX - mapInfo.originX) / mapInfo.resolution);
    }

    /**
     * ✅ 统一坐标系：世界坐标Y -> 栅格坐标Y
     */
    private int worldToGridY(double worldY, MapServerManager.MapInfo mapInfo) {
        return (int) Math.round(mapInfo.height - 1 - (worldY - mapInfo.originY) / mapInfo.resolution);
    }

    /**
     * ✅ 统一坐标系：栅格坐标X -> 世界坐标X
     */
    private double gridToWorldX(int gridX, MapServerManager.MapInfo mapInfo) {
        return mapInfo.originX + gridX * mapInfo.resolution;
    }

    /**
     * ✅ 统一坐标系：栅格坐标Y -> 世界坐标Y
     */

    private double gridToWorldY(int gridY, MapServerManager.MapInfo mapInfo) {
        return mapInfo.originY + (mapInfo.height - 1 - gridY) * mapInfo.resolution;
    }
    /**
     * 检查栅格点是否在地图范围内
     */
    private boolean isValidGridPoint(int gridX, int gridY, MapServerManager.MapInfo mapInfo) {
        return gridX >= 0 && gridX < mapInfo.width && gridY >= 0 && gridY < mapInfo.height;
    }

    /**
     * 检查栅格点是否可通行

    private boolean isGridPointNavigable(int gridX, int gridY, byte[] mapData,
                                         MapServerManager.MapInfo mapInfo) {
        if (!isValidGridPoint(gridX, gridY, mapInfo)) {
            return false;
        }

        int index = gridY * mapInfo.width + gridX;
        byte occupancy = mapData[index];

        // 0表示自由空间，可以通行
        return occupancy == 0;
    }*/
    private boolean isGridPointNavigable(int x, int y, byte[] mapData, MapServerManager.MapInfo mapInfo) {
        if (!isValidGridPoint(x, y, mapInfo)) {
            return false;
        }

        int index = y * mapInfo.width + x;
        byte occupancyValue = mapData[index];

        // 地图值含义：
        // 0   = 自由区域（可通行）
        // 100 = 占用区域（不可通行）
        // -1  = 未知区域（不可通行）

        if (occupancyValue == 0) {
            return true;   // 只有自由区域可通行
        } else if (occupancyValue == 100) {
            return false;  // 占用区域不可通行
        } else {
            return false;  // 未知区域也不可通行（包括-1）
        }
    }
    /**
     * 将地图数据转换为A*算法所需格式
     */
    private int[] convertMapDataForAstar(byte[] mapData) {
        int[] astarData = new int[mapData.length];
        for (int i = 0; i < mapData.length; i++) {
            if (mapData[i] == 0) {
                astarData[i] = 0;        // 可通行
            } else {
                astarData[i] = 100;      // 障碍物，匹配C++的OBSTACLE值
            }
        }
        return astarData;
    }

    /**
     * 计算点到直线的距离（保留原有方法）
     */
    private double calculatePointToLineDistance(PathPoint point, PathPoint lineStart, PathPoint lineEnd) {
        double x0 = point.worldX;
        double y0 = point.worldY;
        double x1 = lineStart.worldX;
        double y1 = lineStart.worldY;
        double x2 = lineEnd.worldX;
        double y2 = lineEnd.worldY;

        double A = x0 - x1;
        double B = y0 - y1;
        double C = x2 - x1;
        double D = y2 - y1;

        double dot = A * C + B * D;
        double lenSq = C * C + D * D;

        if (lenSq == 0) {
            return Math.sqrt(A * A + B * B);
        }

        double param = dot / lenSq;

        double xx, yy;
        if (param < 0) {
            xx = x1;
            yy = y1;
        } else if (param > 1) {
            xx = x2;
            yy = y2;
        } else {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }

        double dx = x0 - xx;
        double dy = y0 - yy;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * 显示Toast消息
     */
    private void showToast(String message) {
        Toast.makeText(context, message, Toast.LENGTH_SHORT).show();
    }

    /**
     * 如果目标点不可通行，则使用 BFS 搜索最近的可通行点（修复版）
     * @param endPoint 原始目标点（栅格坐标）
     * @param mapData 地图数据
     * @param mapInfo 地图信息
     * @return 处理后的可通行目标点（栅格坐标）
     */
    private PathPoint processGoalPointIfBlocked(PathPoint endPoint, byte[] mapData,
                                                MapServerManager.MapInfo mapInfo) {
        int x = endPoint.gridX;
        int y = endPoint.gridY;

        Log.i(TAG, String.format("处理目标点: Grid(%d, %d), 地图尺寸: %dx%d", x, y, mapInfo.width, mapInfo.height));

        // ✅ 步骤1：如果目标点超出地图范围，先吸附到最近的地图边界
        boolean wasOutOfBounds = false;
        if (!isValidGridPoint(x, y, mapInfo)) {
            wasOutOfBounds = true;
            Log.i(TAG, "目标点超出地图范围，执行边界吸附");

            // 吸附到地图边界
            int originalX = x, originalY = y;
            x = Math.max(0, Math.min(x, mapInfo.width - 1));
            y = Math.max(0, Math.min(y, mapInfo.height - 1));

            Log.i(TAG, String.format("边界吸附: (%d,%d) → (%d,%d)", originalX, originalY, x, y));

            debugLogger.updateDebugTextView(String.format("目标点超界，吸附: (%d,%d)→(%d,%d)",
                    originalX, originalY, x, y));
        }

        // ✅ 步骤2：检查吸附后的点是否可通行
        if (isGridPointNavigable(x, y, mapData, mapInfo)) {
            Log.i(TAG, "吸附后的目标点可通行");
            PathPoint result = new PathPoint(x, y);
            result.worldX = gridToWorldX(x, mapInfo);
            result.worldY = gridToWorldY(y, mapInfo);

            if (wasOutOfBounds) {
              //  debugLogger.updateDebugTextView("吸附成功，目标点可通行");
            }

            return result;
        }

        // ✅ 步骤3：吸附后的点仍不可通行，使用BFS搜索最近可通行点
        Log.i(TAG, "吸附后的目标点仍不可通行，开始BFS搜索");
        //debugLogger.updateDebugTextView("开始BFS搜索最近可通行点");

        // 合理的搜索半径（不要设太大）
        int maxSearchRadius = Math.min(50, Math.max(mapInfo.width, mapInfo.height) / 4);
        int[] dx = {-1, 1, 0, 0, -1, -1, 1, 1};
        int[] dy = {0, 0, -1, 1, -1, 1, -1, 1};

        boolean[][] visited = new boolean[mapInfo.height][mapInfo.width];
        java.util.Queue<int[]> queue = new java.util.LinkedList<>();

        // ✅ 从吸附后的点开始搜索（确保在地图范围内）
        queue.offer(new int[]{x, y, 0}); // [x, y, distance]
        visited[y][x] = true;

        while (!queue.isEmpty()) {
            int[] current = queue.poll();
            int cx = current[0];
            int cy = current[1];
            int distance = current[2];

            // 超出搜索半径，停止搜索
            if (distance > maxSearchRadius) {
                break;
            }

            // ✅ 检查当前点是否可通行
            if (isGridPointNavigable(cx, cy, mapData, mapInfo)) {
                Log.i(TAG, String.format("BFS找到可通行点: Grid(%d, %d), 距离: %d", cx, cy, distance));

                PathPoint result = new PathPoint(cx, cy);
                result.worldX = gridToWorldX(cx, mapInfo);
                result.worldY = gridToWorldY(cy, mapInfo);

                return result;
            }

            // 向8个方向扩展
            for (int i = 0; i < 8; i++) {
                int nx = cx + dx[i];
                int ny = cy + dy[i];

                // ✅ 确保新点在地图范围内
                if (!isValidGridPoint(nx, ny, mapInfo)) {
                    continue;
                }

                // 检查是否已访问
                if (visited[ny][nx]) {
                    continue;
                }

                visited[ny][nx] = true;
                queue.offer(new int[]{nx, ny, distance + 1});
            }
        }

        // ✅ 搜索失败，返回吸附后的点（即使不可通行）
        Log.w(TAG, String.format("BFS未找到可通行点，返回吸附点: Grid(%d, %d)", x, y));
        //debugLogger.updateDebugTextView("BFS失败，返回吸附点");

        PathPoint fallback = new PathPoint(x, y);
        fallback.worldX = gridToWorldX(x, mapInfo);
        fallback.worldY = gridToWorldY(y, mapInfo);

        return fallback;
    }
    /**
     * 获取当前路径
     */
    public List<PathPoint> getCurrentPath() {
        return new ArrayList<>(currentPath);
    }

    /**
     * 检查是否正在进行路径规划
     */
    public boolean isPathPlanningInProgress() {
        return pathPlanningInProgress;
    }


    /**
     * 计算路径长度
     */
    public double calculatePathLength(List<PathPoint> path) {
        if (path.size() < 2) {
            return 0.0;
        }

        double totalLength = 0.0;
        for (int i = 1; i < path.size(); i++) {
            PathPoint prev = path.get(i - 1);
            PathPoint curr = path.get(i);

            double dx = curr.worldX - prev.worldX;
            double dy = curr.worldY - prev.worldY;
            totalLength += Math.sqrt(dx * dx + dy * dy);
        }

        return totalLength;
    }



    /**
     * 调用native A*算法
     */
    private native int[] nativeFindPath(int[] mapData, int width, int height,
                                        int startX, int startY, int endX, int endY,
                                        double weightA, double weightB, String distance);

    /**
     * 关闭路径规划管理器
     */
    public void shutdown() {
        pathPlanningInProgress = false;
        executor.shutdown();
        clearPath();
        Log.i(TAG, "PathPlanningManager已关闭");
    }
}