package com.vslam.orbslam3.vslamactivity;

import android.app.AlertDialog;
import android.content.ClipData;
import android.content.ClipboardManager;
import android.content.Context;
import android.graphics.*;
import android.text.InputType;
import android.util.Log;
import android.view.MotionEvent;
import android.view.ViewGroup;
import android.widget.*;

import java.util.ArrayList;
import java.util.List;
import android.view.Gravity;
/**
 * 点云渲染器 - 负责点云数据的可视化和交互
 */
public class PointCloudRenderer {
    private static final String TAG = "PointCloudRenderer";

    // 渲染模式常量
    public static final int MODE_3D_PROJECTION = 0;
    public static final int MODE_TOP_VIEW = 1;
    public static final int MODE_SIDE_VIEW = 2;

    // 成员变量
    private Context context;
    private float[] currentPointCloud = null;
    private final Object pointCloudLock = new Object();
    private boolean showPointCloud = true;
    private int pointCloudRenderMode = MODE_3D_PROJECTION;

    // 3D投影参数
    private float cameraDistance = 5.0f;
    private float rotationX = 0.0f;
    private float rotationY = 0.0f;
    private float lastTouchX, lastTouchY;

    // 渲染配置
    private float maxRenderDistance = 20.0f;
    private int maxRenderPoints = 1000;
    private float pointSizeMultiplier = 1.0f;

    // 性能统计
    private long lastRenderTime = 0;
    private int renderedPointCount = 0;

    // 颜色配置
    private boolean usePointColors = true;
    private int fallbackColor = Color.WHITE;

    // UI组件引用
    private CheckBox showPointCloudCheckBox;
    private Button pointCloudModeButton;
    private Button pointCloudSettingsButton;

    public PointCloudRenderer(Context context) {
        this.context = context;
    }

    /**
     * 初始化点云相关的UI控件
     */
    public void initializePointCloudUI(ViewGroup rootView) {
        // 点云显示控制
        showPointCloudCheckBox = new CheckBox(context);
        showPointCloudCheckBox.setText("显示点云");
        showPointCloudCheckBox.setTextColor(Color.WHITE);
        showPointCloudCheckBox.setChecked(showPointCloud);
        showPointCloudCheckBox.setOnCheckedChangeListener((buttonView, isChecked) -> {
            setShowPointCloud(isChecked);
        });

        // 点云渲染模式切换按钮
        pointCloudModeButton = new Button(context);
        pointCloudModeButton.setText("点云模式");
        pointCloudModeButton.setTextSize(12);
        pointCloudModeButton.setOnClickListener(v -> switchRenderMode());

        // 点云设置按钮
        pointCloudSettingsButton = new Button(context);
        pointCloudSettingsButton.setText("点云设置");
        pointCloudSettingsButton.setTextSize(12);
        pointCloudSettingsButton.setOnClickListener(v -> showPointCloudSettings());

        // 添加点云控制组件到布局
        FrameLayout.LayoutParams pointCloudCheckParams = new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT,
                ViewGroup.LayoutParams.WRAP_CONTENT);
        pointCloudCheckParams.gravity = Gravity.TOP | Gravity.START;
        pointCloudCheckParams.setMargins(20, 500, 0, 0);
        rootView.addView(showPointCloudCheckBox, pointCloudCheckParams);

        FrameLayout.LayoutParams modeButtonParams = new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT,
                ViewGroup.LayoutParams.WRAP_CONTENT);
        modeButtonParams.gravity = Gravity.TOP | Gravity.START;
        modeButtonParams.setMargins(20, 340, 0, 0);
        rootView.addView(pointCloudModeButton, modeButtonParams);

        FrameLayout.LayoutParams settingsButtonParams = new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT,
                ViewGroup.LayoutParams.WRAP_CONTENT);
        settingsButtonParams.gravity = Gravity.TOP | Gravity.START;
        settingsButtonParams.setMargins(20, 180, 0, 0);
        rootView.addView(pointCloudSettingsButton, settingsButtonParams);
    }

    /**
     * 更新点云数据
     */
    public void updatePointCloudData(float[] pointCloud) {
        float[] optimized = optimizePointCloudForRendering(pointCloud);
        synchronized (pointCloudLock) {
            currentPointCloud = optimized;
        }
    }

    /**
     * 绘制点云
     */
    public void drawPointCloud(Canvas canvas) {
        if (!showPointCloud) return;

        long startTime = System.currentTimeMillis();

        synchronized (pointCloudLock) {
            if (currentPointCloud == null || currentPointCloud.length < 6) {
                return;
            }

            Paint pointPaint = new Paint();
            pointPaint.setAntiAlias(true);
            pointPaint.setStyle(Paint.Style.FILL);

            int pointCount = currentPointCloud.length / 6;
            renderedPointCount = 0;

            for (int i = 0; i < pointCount; i++) {
                int baseIndex = i * 6;

                // 获取3D坐标
                float x = currentPointCloud[baseIndex];
                float y = currentPointCloud[baseIndex + 1];
                float z = currentPointCloud[baseIndex + 2];

                // 获取颜色
                float r = currentPointCloud[baseIndex + 3];
                float g = currentPointCloud[baseIndex + 4];
                float b = currentPointCloud[baseIndex + 5];

                // 转换为屏幕坐标
                PointF screenPoint = projectToScreen(x, y, z, canvas.getWidth(), canvas.getHeight());

                if (screenPoint != null) {
                    // 设置点的颜色
                    int color = usePointColors ?
                            Color.rgb((int)(r * 255), (int)(g * 255), (int)(b * 255)) :
                            fallbackColor;
                    pointPaint.setColor(color);

                    // 根据深度调整点的大小
                    float pointSize = getPointSize(z) * pointSizeMultiplier;

                    // 绘制点
                    canvas.drawCircle(screenPoint.x, screenPoint.y, pointSize, pointPaint);
                    renderedPointCount++;
                }
            }
        }

        lastRenderTime = System.currentTimeMillis() - startTime;
    }

    /**
     * 点云设置对话框
     */
    public void showPointCloudSettings() {
        AlertDialog.Builder builder = new AlertDialog.Builder(context);
        builder.setTitle("点云设置");

        String[] options = {
                "切换渲染模式 (" + getRenderModeName() + ")",
                "重置渲染参数",
                "调整最大渲染距离 (" + maxRenderDistance + ")",
                "调整最大渲染点数 (" + maxRenderPoints + ")",
                "调整点大小 (" + pointSizeMultiplier + ")",
                "切换颜色模式 (" + (usePointColors ? "彩色" : "单色") + ")",
                "查看详细状态",
                "输出调试信息"
        };

        builder.setItems(options, (dialog, which) -> {
            switch (which) {
                case 0:
                    switchRenderMode();
                    break;
                case 1:
                    resetRenderParams();
                    break;
                case 2:
                    showDistanceSettingDialog();
                    break;
                case 3:
                    showPointCountSettingDialog();
                    break;
                case 4:
                    showPointSizeSettingDialog();
                    break;
                case 5:
                    setUsePointColors(!usePointColors);
                    Toast.makeText(context, "颜色模式已切换为: " +
                                    (usePointColors ? "彩色" : "单色"),
                            Toast.LENGTH_SHORT).show();
                    break;
                case 6:
                    showPointCloudDetailedStatus();
                    break;
                case 7:
                    logDebugInfo();
                    Toast.makeText(context, "调试信息已输出到日志", Toast.LENGTH_SHORT).show();
                    break;
            }
        });

        builder.setNegativeButton("取消", null);
        builder.show();
    }

    /**
     * 距离设置对话框
     */
    private void showDistanceSettingDialog() {
        AlertDialog.Builder builder = new AlertDialog.Builder(context);
        builder.setTitle("最大渲染距离");

        final EditText input = new EditText(context);
        input.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_DECIMAL);
        input.setText(String.valueOf(maxRenderDistance));
        builder.setView(input);

        builder.setPositiveButton("确定", (dialog, which) -> {
            try {
                float distance = Float.parseFloat(input.getText().toString());
                setMaxRenderDistance(distance);
                Toast.makeText(context, "最大渲染距离已设置为: " + distance, Toast.LENGTH_SHORT).show();
            } catch (NumberFormatException e) {
                Toast.makeText(context, "输入格式错误", Toast.LENGTH_SHORT).show();
            }
        });

        builder.setNegativeButton("取消", null);
        builder.show();
    }

    /**
     * 点数设置对话框
     */
    private void showPointCountSettingDialog() {
        AlertDialog.Builder builder = new AlertDialog.Builder(context);
        builder.setTitle("最大渲染点数");

        final EditText input = new EditText(context);
        input.setInputType(InputType.TYPE_CLASS_NUMBER);
        input.setText(String.valueOf(maxRenderPoints));
        builder.setView(input);

        builder.setPositiveButton("确定", (dialog, which) -> {
            try {
                int points = Integer.parseInt(input.getText().toString());
                setMaxRenderPoints(points);
                Toast.makeText(context, "最大渲染点数已设置为: " + points, Toast.LENGTH_SHORT).show();
            } catch (NumberFormatException e) {
                Toast.makeText(context, "输入格式错误", Toast.LENGTH_SHORT).show();
            }
        });

        builder.setNegativeButton("取消", null);
        builder.show();
    }

    /**
     * 点大小设置对话框
     */
    private void showPointSizeSettingDialog() {
        AlertDialog.Builder builder = new AlertDialog.Builder(context);
        builder.setTitle("点大小倍数");

        final EditText input = new EditText(context);
        input.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_DECIMAL);
        input.setText(String.valueOf(pointSizeMultiplier));
        builder.setView(input);

        builder.setPositiveButton("确定", (dialog, which) -> {
            try {
                float multiplier = Float.parseFloat(input.getText().toString());
                setPointSizeMultiplier(multiplier);
                Toast.makeText(context, "点大小倍数已设置为: " + multiplier, Toast.LENGTH_SHORT).show();
            } catch (NumberFormatException e) {
                Toast.makeText(context, "输入格式错误", Toast.LENGTH_SHORT).show();
            }
        });

        builder.setNegativeButton("取消", null);
        builder.show();
    }

    /**
     * 显示详细状态对话框
     */
    private void showPointCloudDetailedStatus() {
        new AlertDialog.Builder(context)
                .setTitle("点云详细状态")
                .setMessage(getDetailedStatus())
                .setPositiveButton("确定", null)
                .setNeutralButton("复制", (dialog, which) -> {
                    ClipboardManager clipboard = (ClipboardManager) context.getSystemService(Context.CLIPBOARD_SERVICE);
                    ClipData clip = ClipData.newPlainText("点云状态", getDetailedStatus());
                    clipboard.setPrimaryClip(clip);
                    Toast.makeText(context, "已复制到剪贴板", Toast.LENGTH_SHORT).show();
                })
                .show();
    }

    /**
     * 将3D坐标投影到屏幕坐标
     */
    private PointF projectToScreen(float x, float y, float z, int screenWidth, int screenHeight) {
        try {
            switch (pointCloudRenderMode) {
                case MODE_3D_PROJECTION:
                    return project3D(x, y, z, screenWidth, screenHeight);
                case MODE_TOP_VIEW:
                    return projectTopView(x, y, z, screenWidth, screenHeight);
                case MODE_SIDE_VIEW:
                    return projectSideView(x, y, z, screenWidth, screenHeight);
                default:
                    return project3D(x, y, z, screenWidth, screenHeight);
            }
        } catch (Exception e) {
            Log.e(TAG, "投影计算异常", e);
            return null;
        }
    }

    /**
     * 3D透视投影
     */
    private PointF project3D(float x, float y, float z, int screenWidth, int screenHeight) {
        // 应用旋转变换
        float rotatedX = (float) (x * Math.cos(rotationY) - z * Math.sin(rotationY));
        float rotatedY = (float) (y * Math.cos(rotationX) - z * Math.sin(rotationX));
        float rotatedZ = (float) (x * Math.sin(rotationY) + z * Math.cos(rotationY));

        // 透视投影
        float distance = cameraDistance + rotatedZ;
        if (distance <= 0.1f) return null; // 避免除零

        float fov = 60.0f; // 视场角
        float scale = (float) (screenHeight / (2 * Math.tan(Math.toRadians(fov / 2))));

        PointF result = new PointF();
        result.x = screenWidth / 2 + (rotatedX * scale) / distance;
        result.y = screenHeight / 2 - (rotatedY * scale) / distance;

        // 检查是否在屏幕范围内
        if (result.x >= 0 && result.x < screenWidth &&
                result.y >= 0 && result.y < screenHeight) {
            return result;
        }

        return null;
    }

    /**
     * 俯视图投影 (XY平面)
     */
    private PointF projectTopView(float x, float y, float z, int screenWidth, int screenHeight) {
        PointF result = new PointF();
        float scale = 50.0f; // 缩放因子

        result.x = screenWidth / 2 + x * scale;
        result.y = screenHeight / 2 + y * scale;

        // 检查是否在屏幕范围内
        if (result.x >= 0 && result.x < screenWidth &&
                result.y >= 0 && result.y < screenHeight) {
            return result;
        }

        return null;
    }

    /**
     * 侧视图投影 (XZ平面)
     */
    private PointF projectSideView(float x, float y, float z, int screenWidth, int screenHeight) {
        PointF result = new PointF();
        float scale = 50.0f; // 缩放因子

        result.x = screenWidth / 2 + x * scale;
        result.y = screenHeight / 2 - z * scale; // 注意Z轴方向

        // 检查是否在屏幕范围内
        if (result.x >= 0 && result.x < screenWidth &&
                result.y >= 0 && result.y < screenHeight) {
            return result;
        }

        return null;
    }

    /**
     * 根据深度获取点的大小
     */
    private float getPointSize(float z) {
        float minSize = 1.0f;
        float maxSize = 8.0f;
        float normalizedZ = Math.abs(z) / 10.0f; // 归一化深度

        return Math.max(minSize, maxSize - normalizedZ * 3);
    }

    /**
     * 处理触摸事件
     */
    public boolean onTouchEvent(MotionEvent event) {
        switch (event.getAction()) {
            case MotionEvent.ACTION_DOWN:
                lastTouchX = event.getX();
                lastTouchY = event.getY();
                return true;

            case MotionEvent.ACTION_MOVE:
                if (pointCloudRenderMode == MODE_3D_PROJECTION) {
                    float deltaX = event.getX() - lastTouchX;
                    float deltaY = event.getY() - lastTouchY;

                    rotationY += deltaX * 0.01f;
                    rotationX += deltaY * 0.01f;

                    lastTouchX = event.getX();
                    lastTouchY = event.getY();
                    return true;
                }
                break;
        }
        return false;
    }

    /**
     * 切换渲染模式
     */
    public void switchRenderMode() {
        pointCloudRenderMode = (pointCloudRenderMode + 1) % 3;

        String[] modeNames = {"3D投影", "俯视图", "侧视图"};
        Toast.makeText(context, "点云模式: " + modeNames[pointCloudRenderMode], Toast.LENGTH_SHORT).show();
    }

    /**
     * 优化点云数据以提高渲染性能
     */
    private float[] optimizePointCloudForRendering(float[] pointCloud) {
        if (pointCloud == null) return null;

        // 1. 距离过滤
        float[] filtered = filterPointCloudByDistance(pointCloud, maxRenderDistance);

        // 2. 抽样减少点数
        float[] sampled = samplePointCloud(filtered, maxRenderPoints);

        return sampled;
    }

    /**
     * 过滤点云数据（根据距离）
     */
    private float[] filterPointCloudByDistance(float[] pointCloud, float maxDistance) {
        if (pointCloud == null || pointCloud.length < 6) {
            return pointCloud;
        }

        List<Float> filteredPoints = new ArrayList<>();
        int pointCount = pointCloud.length / 6;

        for (int i = 0; i < pointCount; i++) {
            int baseIndex = i * 6;
            float x = pointCloud[baseIndex];
            float y = pointCloud[baseIndex + 1];
            float z = pointCloud[baseIndex + 2];

            float distance = (float) Math.sqrt(x * x + y * y + z * z);
            if (distance <= maxDistance) {
                for (int j = 0; j < 6; j++) {
                    filteredPoints.add(pointCloud[baseIndex + j]);
                }
            }
        }

        float[] result = new float[filteredPoints.size()];
        for (int i = 0; i < filteredPoints.size(); i++) {
            result[i] = filteredPoints.get(i);
        }

        return result;
    }

    /**
     * 对点云数据进行抽样
     */
    private float[] samplePointCloud(float[] pointCloud, int maxPoints) {
        if (pointCloud == null || pointCloud.length < 6) {
            return pointCloud;
        }

        int pointCount = pointCloud.length / 6;
        if (pointCount <= maxPoints) {
            return pointCloud;
        }

        float[] result = new float[maxPoints * 6];
        int step = pointCount / maxPoints;

        for (int i = 0; i < maxPoints; i++) {
            int sourceIndex = i * step * 6;
            int targetIndex = i * 6;

            for (int j = 0; j < 6; j++) {
                result[targetIndex + j] = pointCloud[sourceIndex + j];
            }
        }

        return result;
    }

    // ===== Getter 和 Setter 方法 =====

    public boolean isShowPointCloud() {
        return showPointCloud;
    }

    public void setShowPointCloud(boolean show) {
        this.showPointCloud = show;
    }

    public int getRenderMode() {
        return pointCloudRenderMode;
    }

    public void setRenderMode(int mode) {
        this.pointCloudRenderMode = mode % 3;
    }

    public float getCameraDistance() {
        return cameraDistance;
    }

    public void setCameraDistance(float distance) {
        this.cameraDistance = Math.max(0.1f, distance);
    }

    public float getRotationX() {
        return rotationX;
    }

    public void setRotationX(float rotation) {
        this.rotationX = rotation;
    }

    public float getRotationY() {
        return rotationY;
    }

    public void setRotationY(float rotation) {
        this.rotationY = rotation;
    }

    public float getMaxRenderDistance() {
        return maxRenderDistance;
    }

    public void setMaxRenderDistance(float distance) {
        this.maxRenderDistance = Math.max(0.1f, distance);
    }

    public int getMaxRenderPoints() {
        return maxRenderPoints;
    }

    public void setMaxRenderPoints(int points) {
        this.maxRenderPoints = Math.max(1, points);
    }

    public float getPointSizeMultiplier() {
        return pointSizeMultiplier;
    }

    public void setPointSizeMultiplier(float multiplier) {
        this.pointSizeMultiplier = Math.max(0.1f, multiplier);
    }

    public boolean isUsePointColors() {
        return usePointColors;
    }

    public void setUsePointColors(boolean useColors) {
        this.usePointColors = useColors;
    }

    public int getFallbackColor() {
        return fallbackColor;
    }

    public void setFallbackColor(int color) {
        this.fallbackColor = color;
    }

    // ===== 状态和统计方法 =====

    /**
     * 获取当前点云数据
     */
    public float[] getCurrentPointCloudData() {
        synchronized (pointCloudLock) {
            return currentPointCloud != null ? currentPointCloud.clone() : null;
        }
    }

    /**
     * 检查点云数据是否有效
     */
    public boolean isPointCloudDataValid() {
        synchronized (pointCloudLock) {
            return currentPointCloud != null && currentPointCloud.length >= 6;
        }
    }

    /**
     * 清除点云数据
     */
    public void clearPointCloudData() {
        synchronized (pointCloudLock) {
            currentPointCloud = null;
        }
    }

    /**
     * 获取点云统计信息
     */
    public String getPointCloudStats() {
        synchronized (pointCloudLock) {
            if (currentPointCloud == null) {
                return "点云: 无数据";
            }

            int totalPoints = currentPointCloud.length / 6;
            return String.format("点云: %d个点 (渲染: %d)", totalPoints, renderedPointCount);
        }
    }

    /**
     * 获取渲染性能统计
     */
    public String getRenderStats() {
        return String.format("渲染时间: %dms, 点数: %d", lastRenderTime, renderedPointCount);
    }

    /**
     * 重置渲染参数
     */
    public void resetRenderParams() {
        cameraDistance = 5.0f;
        rotationX = 0.0f;
        rotationY = 0.0f;
        pointSizeMultiplier = 1.0f;
        Toast.makeText(context, "点云渲染参数已重置", Toast.LENGTH_SHORT).show();
    }

    /**
     * 获取渲染模式名称
     */
    public String getRenderModeName() {
        String[] modeNames = {"3D投影", "俯视图", "侧视图"};
        return modeNames[pointCloudRenderMode];
    }

    /**
     * 获取详细状态信息
     */
    public String getDetailedStatus() {
        StringBuilder status = new StringBuilder();
        status.append("点云渲染器状态:\n");
        status.append("显示状态: ").append(showPointCloud ? "开启" : "关闭").append("\n");
        status.append("渲染模式: ").append(getRenderModeName()).append("\n");
        status.append("相机距离: ").append(String.format("%.2f", cameraDistance)).append("\n");
        status.append("旋转角度: X=").append(String.format("%.2f", rotationX))
                .append(", Y=").append(String.format("%.2f", rotationY)).append("\n");
        status.append("最大渲染距离: ").append(maxRenderDistance).append("\n");
        status.append("最大渲染点数: ").append(maxRenderPoints).append("\n");
        status.append("点大小倍数: ").append(pointSizeMultiplier).append("\n");
        status.append("使用点颜色: ").append(usePointColors ? "是" : "否").append("\n");
        status.append(getPointCloudStats()).append("\n");
        status.append(getRenderStats());

        return status.toString();
    }

    /**
     * 输出调试信息
     */
    public void logDebugInfo() {
        Log.d(TAG, getDetailedStatus());

        synchronized (pointCloudLock) {
            if (currentPointCloud != null && currentPointCloud.length >= 6) {
                int pointCount = Math.min(5, currentPointCloud.length / 6);
                Log.d(TAG, "前" + pointCount + "个点的详细信息:");

                for (int i = 0; i < pointCount; i++) {
                    int baseIndex = i * 6;
                    Log.d(TAG, String.format("点%d: 位置(%.2f, %.2f, %.2f) 颜色(%.2f, %.2f, %.2f)",
                            i,
                            currentPointCloud[baseIndex],
                            currentPointCloud[baseIndex + 1],
                            currentPointCloud[baseIndex + 2],
                            currentPointCloud[baseIndex + 3],
                            currentPointCloud[baseIndex + 4],
                            currentPointCloud[baseIndex + 5]
                    ));
                }
            }
        }
    }
}