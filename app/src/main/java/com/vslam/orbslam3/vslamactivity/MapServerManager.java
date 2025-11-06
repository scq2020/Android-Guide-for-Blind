package com.vslam.orbslam3.vslamactivity;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Matrix;
import android.util.Log;
import android.os.Handler;
import android.os.Looper;
import android.widget.ImageView;
import android.widget.Toast;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.CvType;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/**
 * 地图服务器管理器 - 参考ROS map_server实现
 * 功能：加载地图文件并发布/map话题（nav_msgs/OccupancyGrid）+ 界面显示
 */
public class MapServerManager {
    private static final String TAG = "MapServerManager";

    // ✅ 参考map_server的默认参数
    private static final double DEFAULT_OCCUPIED_THRESH = 0.65;
    private static final double DEFAULT_FREE_THRESH = 0.196;
    private static final boolean DEFAULT_NEGATE = false;
    private static final String DEFAULT_FRAME_ID = "map";

    // ✅ 地图显示相关配置
    private static final int COLOR_OCCUPIED = Color.BLACK;      // 占用区域 - 黑色
    private static final int COLOR_FREE = Color.WHITE;         // 自由区域 - 白色
    private static final int COLOR_UNKNOWN = Color.GRAY;       // 未知区域 - 灰色
    private static final int COLOR_BACKGROUND = Color.LTGRAY;  // 背景色

    // 地图数据
    private byte[] mapData;
    private MapInfo mapInfo;
    private boolean mapLoaded = false;

    // ✅ 地图显示相关
    private Bitmap displayBitmap;           // 用于显示的地图位图
    private boolean needUpdateDisplay = true; // 是否需要更新显示
    private float displayScale = 1.0f;      // 显示缩放比例
    private ImageView mapImageView;         // 地图显示控件

    // ✅ 新增：地图缩放控制器
    private MapScaleController scaleController;

    // ROS相关
    private final Context context;
    private RosManager rosManager;
    private final ExecutorService executor = Executors.newSingleThreadExecutor();
    private final Handler mainHandler = new Handler(Looper.getMainLooper());

    // 发布控制
    private boolean isPublishing = false;
    private static final int PUBLISH_INTERVAL = 1000; // 1秒发布一次（锁存话题）
    private DebugLogger debugLogger;

    /**
     * 地图缩放控制类 - 提供更灵活的缩放功能
     */
    public static class MapScaleController {
        private float baseScale = 1.0f;        // 基础缩放比例
        private float additionalScale = 1.0f;  // 额外缩放因子
        private float minScale = 0.1f;         // 最小缩放
        private float maxScale = 50.0f;         // 最大缩放

        // 缩放中心点
        private float pivotX = 0.5f;  // 默认中心点X (0-1)
        private float pivotY = 0.5f;  // 默认中心点Y (0-1)


        /**
         * 设置基础缩放比例
         * @param scale 基础缩放比例
         */
        public void setBaseScale(float scale) {
            this.baseScale = constrainScale(scale);
        }


        /**
         * 获取当前有效缩放比例
         * @return 实际缩放比例
         */
        public float getEffectiveScale() {
            return constrainScale(baseScale * additionalScale);
        }

        /**
         * 限制缩放在有效范围内
         */
        private float constrainScale(float scale) {
            return Math.max(minScale, Math.min(maxScale, scale));
        }

        /**
         * 设置缩放范围
         * @param min 最小缩放比例
         * @param max 最大缩放比例
         */
        public void setScaleRange(float min, float max) {
            if (min > 0 && max > min) {
                this.minScale = min;
                this.maxScale = max;
            }
        }
    }

    /**
     * 地图信息结构体 - 参考nav_msgs/MapMetaData
     */
    public static class MapInfo {
        public long mapLoadTime;        // 地图加载时间
        public double resolution;       // 分辨率 (m/pixel)
        public int width;              // 宽度 (pixels)
        public int height;             // 高度 (pixels)
        public double originX;         // 原点X坐标 (m)
        public double originY;         // 原点Y坐标 (m)
        public double originZ;         // 原点Z坐标 (m)
        public double orientationX;    // 姿态四元数X
        public double orientationY;    // 姿态四元数Y
        public double orientationZ;    // 姿态四元数Z
        public double orientationW;    // 姿态四元数W
        public String frameId;         // 坐标系ID

        public MapInfo() {
            mapLoadTime = System.currentTimeMillis();
            frameId = DEFAULT_FRAME_ID;
            // 默认姿态为单位四元数
            orientationX = 0.0;
            orientationY = 0.0;
            orientationZ = 0.0;
            orientationW = 1.0;
        }
    }

    /**
     * ✅ 地图显示配置类
     */
    public static class MapDisplayConfig {
        public int occupiedColor = COLOR_OCCUPIED;
        public int freeColor = COLOR_FREE;
        public int unknownColor = COLOR_UNKNOWN;
        public int backgroundColor = COLOR_BACKGROUND;
        public float maxDisplayScale = 500.0f;
        public float minDisplayScale = 0.1f;
        public boolean showGrid = false;        // 是否显示网格
        public int gridColor = Color.LTGRAY;    // 网格颜色
        public boolean showOrigin = true;       // 是否显示原点
        public int originColor = Color.RED;     // 原点颜色
    }

    private MapDisplayConfig displayConfig = new MapDisplayConfig();

    /**
     * YAML配置结构体
     */
    private static class YamlConfig {
        String imagePath;
        double resolution;
        double[] origin = new double[3];
        boolean negate;
        double occupiedThresh;
        double freeThresh;

        public YamlConfig() {
            // 设置默认值
            negate = DEFAULT_NEGATE;
            occupiedThresh = DEFAULT_OCCUPIED_THRESH;
            freeThresh = DEFAULT_FREE_THRESH;
            origin[0] = 0.0;
            origin[1] = 0.0;
            origin[2] = 0.0;
        }
    }

    /**
     * 构造函数
     */
    public MapServerManager(Context context, RosManager rosManager) {
        this.context = context;
        this.rosManager = rosManager;
        this.mapInfo = new MapInfo();
        // 初始化DebugLogger
        this.debugLogger = new DebugLogger(context);

        // 初始化缩放控制器
        this.scaleController = new MapScaleController();
        this.scaleController.setScaleRange(
                displayConfig.minDisplayScale,
                displayConfig.maxDisplayScale
        );

        Log.i(TAG, "MapServerManager初始化完成");
    }

    /**
     * ✅ 设置地图显示的ImageView
     * @param imageView 用于显示地图的ImageView控件
     */
    public void setMapImageView(ImageView imageView) {
        this.mapImageView = imageView;
        if (mapLoaded && needUpdateDisplay) {
            updateMapDisplay();
        }
    }

    /**
     * 获取地图缩放控制器
     * @return 缩放控制器
     */
    public MapScaleController getScaleController() {
        return scaleController;
    }

    private void updateDebugInfo(String message) {
        if (debugLogger != null) {
            debugLogger.updateDebugTextView(message);
        }
    }

    /**
     * 加载地图文件 - 参考map_server的loadMapFromYaml
     * @param yamlFilePath YAML配置文件路径
     */
    public void loadMapFromYaml(String yamlFilePath) {
        executor.execute(() -> {
            try {
                Log.i(TAG, "开始加载地图文件: " + yamlFilePath);

                // 1. 解析YAML配置文件
                YamlConfig config = parseYamlFile(yamlFilePath);
                if (config == null) {
                    throw new Exception("解析YAML文件失败");
                }

                // 2. 构建图像文件完整路径
                String imagePath = config.imagePath;
                if (!new File(imagePath).isAbsolute()) {
                    // 相对路径，相对于YAML文件位置
                    File yamlFile = new File(yamlFilePath);
                    File imageFile = new File(yamlFile.getParent(), imagePath);
                    imagePath = imageFile.getAbsolutePath();
                }

                // 3. 加载图像并转换为占用栅格
                loadMapFromImage(imagePath, config);

                // 4. ✅ 创建显示用的位图
                if (mapLoaded) {
                    createDisplayBitmap();

                    // 5. 开始发布地图
                    startPublishing();

                    mainHandler.post(() -> {
                        // ✅ 更新界面显示
                        updateMapDisplay();

                        //Toast.makeText(context, "地图加载成功", Toast.LENGTH_SHORT).show();

                    });
                }

            } catch (Exception e) {
                Log.e(TAG, "加载地图失败: " + e.getMessage(), e);
                mainHandler.post(() -> {
                    //Toast.makeText(context, "地图加载失败: " + e.getMessage(),
                            //Toast.LENGTH_LONG).show();
                });
            }
        });
    }

    /**
     * 解析YAML配置文件 - 简化版YAML解析
     * @param yamlFilePath YAML文件路径
     * @return 配置对象
     */
    private YamlConfig parseYamlFile(String yamlFilePath) {
        try {
            YamlConfig config = new YamlConfig();

            BufferedReader reader = new BufferedReader(new FileReader(yamlFilePath));
            String line;

            while ((line = reader.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#")) {
                    continue;
                }

                String[] parts = line.split(":");
                if (parts.length < 2) continue;

                String key = parts[0].trim();
                String value = parts[1].trim();

                switch (key) {
                    case "image":
                        config.imagePath = value;
                        break;
                    case "resolution":
                        config.resolution = Double.parseDouble(value);
                        break;
                    case "origin":
                        // 解析origin数组: [x, y, theta]
                        String originStr = value.replaceAll("[\\[\\]]", "");
                        String[] originParts = originStr.split(",");
                        if (originParts.length >= 3) {
                            config.origin[0] = Double.parseDouble(originParts[0].trim());
                            config.origin[1] = Double.parseDouble(originParts[1].trim());
                            config.origin[2] = Double.parseDouble(originParts[2].trim());
                        }
                        break;
                    case "negate":
                        config.negate = Boolean.parseBoolean(value);
                        break;
                    case "occupied_thresh":
                        config.occupiedThresh = Double.parseDouble(value);
                        break;
                    case "free_thresh":
                        config.freeThresh = Double.parseDouble(value);
                        break;
                }
            }

            reader.close();
            return config;

        } catch (Exception e) {
            Log.e(TAG, "解析YAML文件失败: " + e.getMessage(), e);
            return null;
        }
    }

    /**
     * ✅ 修改：使用OpenCV读取PGM格式图像文件
     * @param imagePath 图像文件路径
     * @param config YAML配置
     */
    private void loadMapFromImage(String imagePath, YamlConfig config) throws Exception {
        Log.i(TAG, "使用OpenCV加载图像文件: " + imagePath);

        // 1. 检查文件是否存在
        File imageFile = new File(imagePath);
        if (!imageFile.exists()) {
            updateDebugInfo("图像文件不存在: " + imagePath);
            throw new Exception("图像文件不存在: " + imagePath);
        }

        // 2. ✅ 使用OpenCV读取图像（支持PGM格式）
        Mat imageMat = Imgcodecs.imread(imagePath, Imgcodecs.IMREAD_GRAYSCALE);
        if (imageMat.empty()) {
            updateDebugInfo("OpenCV无法读取图像文件: " + imagePath);
            throw new Exception("OpenCV无法读取图像文件: " + imagePath);
        }

        Log.i(TAG, String.format("OpenCV成功读取图像: %dx%d, 类型: %d",
                imageMat.cols(), imageMat.rows(), imageMat.type()));

        // 3. ✅ 将OpenCV Mat转换为Bitmap（如果需要进一步处理）
        Bitmap bitmap = null;
        try {
            // 确保Mat是8位灰度图
            Mat processedMat = new Mat();
            if (imageMat.type() != CvType.CV_8UC1) {
                imageMat.convertTo(processedMat, CvType.CV_8UC1);
            } else {
                processedMat = imageMat.clone();
            }

            // 转换为ARGB格式用于Bitmap
            Mat rgbMat = new Mat();
            Imgproc.cvtColor(processedMat, rgbMat, Imgproc.COLOR_GRAY2RGBA);

            bitmap = Bitmap.createBitmap(rgbMat.cols(), rgbMat.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(rgbMat, bitmap);

            // 清理临时Mat
            processedMat.release();
            rgbMat.release();

            Log.i(TAG, String.format("成功转换为Bitmap: %dx%d", bitmap.getWidth(), bitmap.getHeight()));

        } catch (Exception e) {
            Log.e(TAG, "Mat转Bitmap失败: " + e.getMessage(), e);
            throw new Exception("Mat转Bitmap失败: " + e.getMessage());
        }

        // 4. 设置地图信息
        mapInfo.width = imageMat.cols();
        mapInfo.height = imageMat.rows();
        mapInfo.resolution = config.resolution;
        mapInfo.originX = config.origin[0];
        mapInfo.originY = config.origin[1];
        mapInfo.originZ = 0.0;

        // 设置姿态（从yaw角度计算四元数）
        double yaw = config.origin[2];
        mapInfo.orientationX = 0.0;
        mapInfo.orientationY = 0.0;
        mapInfo.orientationZ = Math.sin(yaw / 2.0);
        mapInfo.orientationW = Math.cos(yaw / 2.0);

        // 5. ✅ 直接从OpenCV Mat提取像素数据进行处理
        mapData = new byte[mapInfo.width * mapInfo.height];

        // 从OpenCV Mat直接获取像素数据
        byte[] grayData = new byte[mapInfo.width * mapInfo.height];
        imageMat.get(0, 0, grayData);

        for (int j = 0; j < mapInfo.height; j++) {
            for (int i = 0; i < mapInfo.width; i++) {
                // ✅ 直接使用原始索引，不翻转
                int pixelValue = grayData[j * mapInfo.width + i] & 0xFF;

                // 如果negate为true，反转黑白
                if (config.negate) {
                    pixelValue = 255 - pixelValue;
                }

                // 计算占用概率
                double occ = (255.0 - pixelValue) / 255.0;

                byte occupancyValue;
                if (occ > config.occupiedThresh) {
                    occupancyValue = 100;  // 占用
                } else if (occ < config.freeThresh) {
                    occupancyValue = 0;    // 自由
                } else {
                    occupancyValue = -1;   // 未知
                }

                // ✅ 直接使用原始索引
                int index = j * mapInfo.width + i;
                mapData[index] = occupancyValue;
            }
        }

        // 6. ✅ 清理资源
        if (bitmap != null) {
            bitmap.recycle();
        }
        imageMat.release();

        mapLoaded = true;
        mapInfo.mapLoadTime = System.currentTimeMillis();

        Log.i(TAG, String.format("地图数据转换完成: %dx%d, 分辨率: %.3f m/pixel",
                mapInfo.width, mapInfo.height, mapInfo.resolution));
    }

    /**
     * ✅ 创建用于显示的位图
     */
    private void createDisplayBitmap() {
        if (!mapLoaded || mapData == null) {
            Log.w(TAG, "地图未加载，无法创建显示位图");
            return;
        }

        try {
            Log.i(TAG, "开始创建地图显示位图...");

            // 创建位图
            displayBitmap = Bitmap.createBitmap(mapInfo.width, mapInfo.height, Bitmap.Config.ARGB_8888);

            // 创建像素数组
            int[] pixels = new int[mapInfo.width * mapInfo.height];

            // 转换占用栅格数据为像素颜色
            for (int i = 0; i < mapData.length; i++) {
                byte occupancyValue = mapData[i];
                int color;

                if (occupancyValue == 100) {
                    color = displayConfig.occupiedColor;    // 占用 - 黑色
                } else if (occupancyValue == 0) {
                    color = displayConfig.freeColor;        // 自由 - 白色
                } else {
                    color = displayConfig.unknownColor;     // 未知 - 灰色
                }

                pixels[i] = color;
            }

            // 设置像素到位图
            displayBitmap.setPixels(pixels, 0, mapInfo.width, 0, 0, mapInfo.width, mapInfo.height);

            // ✅ 如果需要显示网格或原点，进行额外绘制
            if (displayConfig.showGrid || displayConfig.showOrigin) {
                drawMapAnnotations();
            }

            needUpdateDisplay = false;

            Log.i(TAG, String.format("地图显示位图创建完成: %dx%d",
                    displayBitmap.getWidth(), displayBitmap.getHeight()));

        } catch (Exception e) {
            Log.e(TAG, "创建显示位图失败: " + e.getMessage(), e);
        }
    }

    /**
     * ✅ 在地图上绘制附加标注（网格、原点等）
     */
    private void drawMapAnnotations() {
        if (displayBitmap == null) return;

        Canvas canvas = new Canvas(displayBitmap);
        Paint paint = new Paint();
        paint.setAntiAlias(true);

        // 绘制网格
        if (displayConfig.showGrid) {
            paint.setColor(displayConfig.gridColor);
            paint.setStrokeWidth(1.0f);
            paint.setStyle(Paint.Style.STROKE);

            // 每10个像素绘制一条网格线
            int gridSpacing = 10;
            for (int x = 0; x < mapInfo.width; x += gridSpacing) {
                canvas.drawLine(x, 0, x, mapInfo.height, paint);
            }
            for (int y = 0; y < mapInfo.height; y += gridSpacing) {
                canvas.drawLine(0, y, mapInfo.width, y, paint);
            }
        }

        // 绘制原点
        if (displayConfig.showOrigin) {
            paint.setColor(displayConfig.originColor);
            paint.setStyle(Paint.Style.FILL);

            // ✅ 修复：统一坐标转换，不翻转
            float originPixelX = (float) (-mapInfo.originX / mapInfo.resolution);
            float originPixelY = (float) (mapInfo.height - (-mapInfo.originY / mapInfo.resolution));

            // 确保原点在图像范围内
            if (originPixelX >= 0 && originPixelX < mapInfo.width &&
                    originPixelY >= 0 && originPixelY < mapInfo.height) {

                // 绘制十字形原点标记
                float crossSize = 5.0f;
                canvas.drawLine(originPixelX - crossSize, originPixelY,
                        originPixelX + crossSize, originPixelY, paint);
                canvas.drawLine(originPixelX, originPixelY - crossSize,
                        originPixelX, originPixelY + crossSize, paint);

                // 绘制圆点
                canvas.drawCircle(originPixelX, originPixelY, 3.0f, paint);
            }
        }
    }

    /**
     * ✅ 更新地图显示 - 修改为使用缩放控制器
     */
    private void updateMapDisplay() {
        if (mapImageView == null || !mapLoaded || displayBitmap == null) {
            return;
        }

        mainHandler.post(() -> {
            try {
                // 获取当前缩放比例
                float scale = scaleController.getEffectiveScale();

                // 创建Matrix进行变换
                Matrix matrix = new Matrix();

                // 获取ImageView和地图尺寸
                int viewWidth = mapImageView.getWidth();
                int viewHeight = mapImageView.getHeight();
                int bitmapWidth = displayBitmap.getWidth();
                int bitmapHeight = displayBitmap.getHeight();

                // 计算居中所需的平移
                float translateX = (viewWidth - bitmapWidth * scale) / 2f;
                float translateY = (viewHeight - bitmapHeight * scale) / 2f;

                // 先应用缩放，再应用平移
                matrix.postScale(scale, scale);
                matrix.postTranslate(translateX, translateY);

                // 设置到ImageView
                mapImageView.setImageBitmap(displayBitmap);
                mapImageView.setImageMatrix(matrix);
                mapImageView.setScaleType(ImageView.ScaleType.MATRIX);

                Log.d(TAG, String.format("地图显示已更新，缩放比例: %.2f, 平移: (%.1f, %.1f)",
                        scale, translateX, translateY));

            } catch (Exception e) {
                Log.e(TAG, "更新地图显示失败: " + e.getMessage(), e);
            }
        });
    }


    /**
     * ✅ 获取地图显示位图（用于其他用途）
     * @return 地图显示位图的副本
     */
    public Bitmap getMapDisplayBitmap() {
        if (displayBitmap == null) {
            return null;
        }
        return displayBitmap.copy(displayBitmap.getConfig(), false);
    }

    /**
     * ✅ 刷新地图显示
     */
    public void refreshMapDisplay() {
        if (mapLoaded) {
            needUpdateDisplay = true;
            createDisplayBitmap();
            updateMapDisplay();
        }
    }

    /**
     * ✅ 统一坐标系：将地图坐标转换为图像坐标
     */
    public float[] mapToImageCoordinates(double mapX, double mapY) {
        if (!mapLoaded) {
            return null;
        }

        // ✅ 统一转换公式：直接线性转换，不翻转
        float imageX = (float) ((mapX - mapInfo.originX) / mapInfo.resolution);
        float imageY = (float) (mapInfo.height - 1 - (mapY - mapInfo.originY) / mapInfo.resolution);

        // 边界检查
        if (imageX < 0 || imageX >= mapInfo.width ||
                imageY < 0 || imageY >= mapInfo.height) {
            return null;
        }

        return new float[]{imageX, imageY};
    }

    /**
     * ✅ 统一坐标系：将图像坐标转换为地图坐标
     */
    public double[] imageToMapCoordinates(float imageX, float imageY) {
        if (!mapLoaded) {
            return null;
        }

        // ✅ 统一转换公式：直接线性转换，不翻转
        double mapX = mapInfo.originX + imageX * mapInfo.resolution;
        double mapY = mapInfo.originY + (mapInfo.height - 1 - imageY) * mapInfo.resolution;

        return new double[]{mapX, mapY};
    }
    /**
     * ✅ 将地图坐标转换为屏幕坐标
     * @param mapX 地图X坐标 (米)
     * @param mapY 地图Y坐标 (米)
     * @return 屏幕坐标 [screenX, screenY]
     */
    public float[] mapToScreenCoordinates(double mapX, double mapY) {
        if (!mapLoaded) {
            return null;
        }

        // 转换为图像坐标
        float imageX = (float) ((mapX - mapInfo.originX) / mapInfo.resolution);
        float imageY = (float) (mapInfo.height - (mapY - mapInfo.originY) / mapInfo.resolution);

        // 考虑缩放比例
        float scale = scaleController.getEffectiveScale();

        // 考虑缩放中心点
        float pivotX = scaleController.pivotX * displayBitmap.getWidth();
        float pivotY = scaleController.pivotY * displayBitmap.getHeight();

        // 应用缩放变换
        float screenX = (imageX - pivotX) * scale + pivotX;
        float screenY = (imageY - pivotY) * scale + pivotY;

        return new float[]{screenX, screenY};
    }

    /**
     * 开始发布地图到/map话题
     */
    private void startPublishing() {
        if (isPublishing || !mapLoaded) {
            return;
        }

        isPublishing = true;
        Log.i(TAG, "开始发布地图到/grid_map话题");

        // 立即发布一次
        publishMap();

        // 定期发布（锁存话题特性）
        mainHandler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if (isPublishing && mapLoaded) {
                    publishMap();
                    mainHandler.postDelayed(this, PUBLISH_INTERVAL);
                }
            }
        }, PUBLISH_INTERVAL);
    }

    /**
     * 发布地图到ROS话题（修改后的版本）
     */
    private void publishMap() {
        if (rosManager == null || !rosManager.isConnected()) {
            Log.w(TAG, "ROS未连接，无法发布地图");
            return;
        }

        if (!isRosPublisherReady()) {
            Log.w(TAG, "ROS地图发布器未就绪");
            return;
        }

        try {
            // 通过RosManager发布地图
            rosManager.publishMap(mapData, mapInfo);

            Log.d(TAG, "地图发布成功到/grid_map话题");

        } catch (Exception e) {
            Log.e(TAG, "发布地图失败: " + e.getMessage(), e);
        }
    }

    /**
     * ✅ 检查ROS发布器是否就绪
     * @return 发布器状态
     */
    public boolean isRosPublisherReady() {
        return rosManager != null && rosManager.isMapPublisherReady();
    }

    /**
     * ✅ 更新地图显示位图（供外部调用）
     * @param bitmap 要显示的位图
     */
    public void updateMapDisplayBitmap(Bitmap bitmap) {
        if (mapImageView != null && bitmap != null) {
            mainHandler.post(() -> {
                try {
                    mapImageView.setImageBitmap(bitmap);
                    Log.d(TAG, "外部位图更新成功");
                } catch (Exception e) {
                    Log.e(TAG, "更新外部位图失败: " + e.getMessage(), e);
                }
            });
        } else {
            Log.w(TAG, "MapImageView未设置或位图为空，无法更新显示");
        }
    }



    /**
     * ✅ 检查ImageView是否已设置
     * @return ImageView设置状态
     */
    public boolean isImageViewSet() {
        return mapImageView != null;
    }

    /**
     * 停止发布地图
     */
    public void stopPublishing() {
        isPublishing = false;
        mainHandler.removeCallbacksAndMessages(null);
        Log.i(TAG, "停止发布地图");
    }

    /**
     * 获取地图信息
     */
    public MapInfo getMapInfo() {
        return mapInfo;
    }

    /**
     * 检查地图是否已加载
     */
    public boolean isMapLoaded() {
        return mapLoaded;
    }

    /**
     * ✅ 获取地图数据的副本（用于调试）
     * @return 地图数据副本
     */
    public byte[] getMapDataCopy() {
        if (!mapLoaded || mapData == null) {
            return null;
        }

        byte[] copy = new byte[mapData.length];
        System.arraycopy(mapData, 0, copy, 0, mapData.length);
        return copy;
    }

    /**
     * ✅ 修改：关闭地图服务器（包括显示资源清理）
     */
    public void shutdown() {
        stopPublishing();
        executor.shutdown();
        mapLoaded = false;
        mapData = null;

        // 清理显示资源
        if (displayBitmap != null && !displayBitmap.isRecycled()) {
            displayBitmap.recycle();
            displayBitmap = null;
        }

        mapImageView = null;

        Log.i(TAG, "MapServerManager已关闭（包括显示资源）");
    }
}