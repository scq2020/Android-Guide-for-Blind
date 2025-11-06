// GridMapBuilder.java - 完全重写，参照C++实现
package com.vslam.orbslam3.vslamactivity;

import android.util.Log;
import java.util.Arrays;

public class GridMapBuilder {
    private static final String TAG = "GridMapBuilder";

    // ✅ 参照C++参数
    private static final float SCALE_FACTOR = 30.0f;      // 20 (原C++中的scale_factor)
    private static final float CLOUD_MAX_Y = 10.0f;      // 10
    private static final float CLOUD_MIN_Y = -10.0f;     // -10
    private static final float CLOUD_MAX_X = 10.0f;      // 10
    private static final float CLOUD_MIN_X = -10.0f;     // -10
    private static final float FREE_THRESH = 0.65f;      // 0.65
    private static final float OCCUPIED_THRESH = 0.1f;   // 0.1
    private static final int VISIT_THRESH = 100;         // 100

    // ✅ 栅格地图变量（参照C++）
    private int[][] globalOccupiedCounter;
    private int[][] globalVisitCounter;
    private byte[][] gridMapData;
    private float normFactorY, normFactorX;
    private float gridMaxY, gridMinY, gridMaxX, gridMinX;
    private int height, width;

    // ✅ 地图信息
    private GridMapInfo mapInfo;
    private boolean initialized = false;

    public static class GridMapInfo {
        public int width;
        public int height;
        public float resolution;
        public float originX;
        public float originY;
        public long timestamp;

        public GridMapInfo(int w, int h, float res, float ox, float oy) {
            width = w; height = h; resolution = res;
            originX = ox; originY = oy;
            timestamp = System.currentTimeMillis();
        }
    }

    public GridMapBuilder() {
        initializeGrid();
    }

    // ✅ 初始化栅格（参照C++的main函数逻辑）
    private void initializeGrid() {
        // 计算栅格参数（完全按照C++逻辑）
        gridMaxY = CLOUD_MAX_Y * SCALE_FACTOR;
        gridMinY = CLOUD_MIN_Y * SCALE_FACTOR;
        gridMaxX = CLOUD_MAX_X * SCALE_FACTOR;
        gridMinX = CLOUD_MIN_X * SCALE_FACTOR;

        height = (int)(gridMaxY - gridMinY);
        width = (int)(gridMaxX - gridMinX);

        normFactorY = (float)(height - 1) / (gridMaxY - gridMinY);
        normFactorX = (float)(width - 1) / (gridMaxX - gridMinX);

        // ✅ 初始化数组（参照C++）
        globalOccupiedCounter = new int[height][width];
        globalVisitCounter = new int[height][width];
        gridMapData = new byte[height][width];

        // 清零所有计数器
        for (int r = 0; r < height; r++) {
            Arrays.fill(globalOccupiedCounter[r], 0);
            Arrays.fill(globalVisitCounter[r], 0);
            Arrays.fill(gridMapData[r], (byte)50); // 未知区域初始值50
        }

        // ✅ 创建地图信息（按照C++的grid_map_msg设置）
        mapInfo = new GridMapInfo(
                width, height,
                1.0f / SCALE_FACTOR,  // resolution = 1/scale_factor
                CLOUD_MIN_X,          // originX
                CLOUD_MIN_Y           // originY
        );

        initialized = true;
        Log.i(TAG, String.format("网格地图初始化完成: %dx%d, 分辨率: %.3f m/pix",
                width, height, mapInfo.resolution));
    }

    // ✅ 坐标转换（完全按照C++的toGrid函数）
    private void toGrid(double x, double y, int[] result) {
        result[0] = (int)Math.floor((x * SCALE_FACTOR - gridMinX) * normFactorX); // gx
        result[1] = (int)Math.floor(height - 1 - (y * SCALE_FACTOR - gridMinY) * normFactorY); // gy
    }

    // ✅ 处理地图点（参照C++的processMapPt函数）
    private void processMapPoint(float ptx, float pty, int kfx, int kfy) {
        int[] ptGrid = new int[2];
        toGrid(ptx, pty, ptGrid);
        int ptgx = ptGrid[0], ptgy = ptGrid[1];

        if (ptgx < 0 || ptgx >= width || ptgy < 0 || ptgy >= height) return;

        // 增加占用计数
        globalOccupiedCounter[ptgy][ptgx]++;

        // ✅ Bresenham算法 - 标记从关键帧到地图点的路径为自由空间
        int dx = Math.abs(kfx - ptgx), dy = Math.abs(kfy - ptgy);
        int sx = ptgx < kfx ? 1 : -1, sy = ptgy < kfy ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2, e2;

        int currentX = ptgx, currentY = ptgy;

        while (true) {
            // 增加访问计数（沿路径的每个点都是自由空间）
            globalVisitCounter[currentY][currentX]++;
            if (currentX == kfx && currentY == kfy) break;

            e2 = err;
            if (e2 > -dx) { err -= dy; currentX += sx; }
            if (e2 < dy) { err += dx; currentY += sy; }
        }
    }

    // ✅ 生成栅格地图（参照C++的getGridMap函数）
    private void generateGridMap() {
        for (int r = 0; r < height; r++) {
            for (int c = 0; c < width; c++) {
                int visits = globalVisitCounter[r][c];
                int occs = globalOccupiedCounter[r][c];

                float prob;
                if (visits <= VISIT_THRESH) {
                    prob = 0.5f; // 未知区域
                } else {
                    prob = 1.0f - (float)occs / visits;
                }

                // 转换为ROS OccupancyGrid格式：0-100，-1为未知
                int occupancyValue = (int)((1 - prob) * 100);
                gridMapData[r][c] = (byte)occupancyValue;
            }
        }

        mapInfo.timestamp = System.currentTimeMillis();
    }

    // ✅ 更新栅格地图（参照C++的updateGridMap函数）
    public synchronized void updateGridMap(float[] keyFrameData) {
        if (!initialized || keyFrameData == null || keyFrameData.length < 7) {
            Log.w(TAG, "网格地图未初始化或关键帧数据无效");
            return;
        }

        try {
            // 关键帧位姿在前7个元素：[x, y, z, qx, qy, qz, qw]
            float kfX = keyFrameData[0];
            float kfY = keyFrameData[1];

            int[] kfGrid = new int[2];
            toGrid(kfX, kfY, kfGrid);
            int kfgx = kfGrid[0], kfgy = kfGrid[1];

            if (kfgy < 0 || kfgy >= height || kfgx < 0 || kfgx >= width) {
                Log.w(TAG, "关键帧位置超出地图范围");
                return;
            }

            // 处理所有地图点（从第7个元素开始，每3个元素为一个点）
            int numPoints = (keyFrameData.length - 7) / 3;
            for (int i = 0; i < numPoints; i++) {
                int baseIndex = 7 + i * 3;
                float ptX = keyFrameData[baseIndex];
                float ptY = keyFrameData[baseIndex + 1];
                // float ptZ = keyFrameData[baseIndex + 2]; // Z坐标暂时不用

                processMapPoint(ptX, ptY, kfgx, kfgy);
            }

            // 重新生成栅格地图
            generateGridMap();

            Log.d(TAG, String.format("更新网格地图：关键帧(%.2f,%.2f) -> 网格(%d,%d), 处理了%d个地图点",
                    kfX, kfY, kfgx, kfgy, numPoints));

        } catch (Exception e) {
            Log.e(TAG, "更新网格地图时出错: " + e.getMessage(), e);
        }
    }

    // ✅ 重置栅格地图（参照C++的resetGridMap函数）
    public synchronized void resetGridMap(float[] allKeyframesData) {
        if (!initialized || allKeyframesData == null || allKeyframesData.length < 1) {
            Log.w(TAG, "无法重置网格地图：数据无效");
            return;
        }

        try {
            // 清零所有计数器
            for (int r = 0; r < height; r++) {
                Arrays.fill(globalOccupiedCounter[r], 0);
                Arrays.fill(globalVisitCounter[r], 0);
            }

            // 解析所有关键帧数据（按照C++格式）
            int nKf = (int)allKeyframesData[0];
            Log.i(TAG, "重置网格地图，包含 " + nKf + " 个关键帧");

            int index = 1;
            for (int i = 0; i < nKf; i++) {
                // 关键帧位姿
                float kfX = allKeyframesData[index++];
                float kfY = allKeyframesData[index++];
                float kfZ = allKeyframesData[index++];
                index += 4; // 跳过四元数

                // 地图点数量
                int nPts = (int)allKeyframesData[index++];

                int[] kfGrid = new int[2];
                toGrid(kfX, kfY, kfGrid);
                int kfgx = kfGrid[0], kfgy = kfGrid[1];

                if (kfgy >= 0 && kfgy < height && kfgx >= 0 && kfgx < width) {
                    // 处理该关键帧的所有地图点
                    for (int j = 0; j < nPts; j++) {
                        float ptX = allKeyframesData[index++];
                        float ptY = allKeyframesData[index++];
                        float ptZ = allKeyframesData[index++];

                        processMapPoint(ptX, ptY, kfgx, kfgy);
                    }
                } else {
                    // 跳过这个关键帧的地图点
                    index += nPts * 3;
                }
            }

            // 重新生成栅格地图
            generateGridMap();
            Log.i(TAG, "网格地图重置完成");

        } catch (Exception e) {
            Log.e(TAG, "重置网格地图时出错: " + e.getMessage(), e);
        }
    }

    // ✅ 获取地图数据
    public synchronized byte[][] getMapData() {
        if (!initialized) return null;

        // 返回地图数据的副本
        byte[][] result = new byte[height][width];
        for (int r = 0; r < height; r++) {
            System.arraycopy(gridMapData[r], 0, result[r], 0, width);
        }
        return result;
    }

    public GridMapInfo getMapInfo() {
        return mapInfo;
    }

    public boolean isInitialized() {
        return initialized;
    }

    // ✅ 检查某个位置是否被占用
    public synchronized boolean isOccupied(float x, float y) {
        if (!initialized) return false;

        int[] grid = new int[2];
        toGrid(x, y, grid);
        int gx = grid[0], gy = grid[1];

        if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
            return false;
        }

        return gridMapData[gy][gx] >= 65; // 占用阈值
    }
}