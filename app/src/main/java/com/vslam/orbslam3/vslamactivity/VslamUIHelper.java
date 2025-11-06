//VslamUIHelper.java文件 - 移除ROS发布按钮，默认发布
package com.vslam.orbslam3.vslamactivity;

import android.app.AlertDialog;
import android.content.ClipData;
import android.content.ClipboardManager;
import android.content.Context;
import android.graphics.*;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.*;
import android.widget.*;

import java.util.HashMap;
import java.util.List;

public class VslamUIHelper {
    private static final String TAG = "VslamUIHelper";

    private VslamActivity activity;
    private TrajectoryView trajectoryView;
    private GridMapView gridMapView; // ✅ 栅格地图视图
    private Button diagButton;
    private Button mapDisplayButton; // ✅ 地图显示开关按钮

    private boolean isMapVisible = false; // ✅ 地图是否可见

    public VslamUIHelper(VslamActivity activity) {
        this.activity = activity;
    }

    public void setupAllUI() {
        addTrajectoryView();
        addGridMapView(); // ✅ 添加栅格地图视图
        addDiagnosticButton();
        addMapDisplayButton(); // ✅ 添加地图显示按钮
        // ✅ 移除了 addRosPublishButton() 调用
    }

    private void addTrajectoryView() {
        trajectoryView = new TrajectoryView(activity);
        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT);

        trajectoryView.setZ(10f);

        ViewGroup rootView = activity.findViewById(android.R.id.content);
        rootView.addView(trajectoryView, params);
        trajectoryView.setVisibility(View.VISIBLE);
    }

    // ✅ 添加右侧分屏的栅格地图视图
    private void addGridMapView() {
        gridMapView = new GridMapView(activity);

        // ✅ 右侧分屏显示，占屏幕右侧40%
        DisplayMetrics displayMetrics = activity.getResources().getDisplayMetrics();
        int screenWidth = displayMetrics.widthPixels;
        int mapWidth = (int)(screenWidth * 0.4f); // 40%屏幕宽度

        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(
                mapWidth, ViewGroup.LayoutParams.MATCH_PARENT);
        params.gravity = Gravity.END | Gravity.CENTER_VERTICAL; // 右侧显示
        params.setMargins(0, 60, 260, 100); // 顶部和底部留空间

        gridMapView.setZ(15f); // 比轨迹视图层级高

        ViewGroup rootView = activity.findViewById(android.R.id.content);
        rootView.addView(gridMapView, params);
        gridMapView.setVisibility(View.GONE); // 初始隐藏
    }

    // ✅ 地图显示开关按钮
    private void addMapDisplayButton() {
        mapDisplayButton = new Button(activity);
        mapDisplayButton.setText("显示地图");
        mapDisplayButton.setTextSize(12);
        mapDisplayButton.setOnClickListener(v -> toggleMapDisplay());

        ViewGroup rootView = activity.findViewById(android.R.id.content);
        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
        params.gravity = Gravity.TOP | Gravity.START;
        params.setMargins(230, 865, 0, 0);
        rootView.addView(mapDisplayButton, params);
    }

    // ✅ 切换地图显示
    private void toggleMapDisplay() {
        if (isMapVisible) {
            // 隐藏地图
            gridMapView.setVisibility(View.GONE);
            mapDisplayButton.setText("显示地图");
            isMapVisible = false;

            // 轨迹视图恢复全屏
            updateTrajectoryLayout(true);
        } else {
            // 显示地图
            gridMapView.setVisibility(View.VISIBLE);
            mapDisplayButton.setText("隐藏地图");
            isMapVisible = true;

            // 轨迹视图调整为左侧60%
            updateTrajectoryLayout(false);
        }
    }

    // ✅ 更新轨迹视图布局
    private void updateTrajectoryLayout(boolean fullScreen) {
        if (trajectoryView == null) return;

        ViewGroup.LayoutParams layoutParams = trajectoryView.getLayoutParams();
        if (layoutParams instanceof FrameLayout.LayoutParams) {
            FrameLayout.LayoutParams params = (FrameLayout.LayoutParams) layoutParams;

            if (fullScreen) {
                // 全屏显示
                params.width = ViewGroup.LayoutParams.MATCH_PARENT;
                params.gravity = Gravity.CENTER;
                params.setMargins(0, 0, 0, 0);
            } else {
                // 左侧60%显示
                DisplayMetrics displayMetrics = activity.getResources().getDisplayMetrics();
                int screenWidth = displayMetrics.widthPixels;
                params.width = (int)(screenWidth * 0.6f); // 60%屏幕宽度
                params.gravity = Gravity.START | Gravity.CENTER_VERTICAL;
                params.setMargins(0, 0, 0, 0);
            }

            trajectoryView.setLayoutParams(params);
            trajectoryView.requestLayout();
        }
    }

    // ✅ 更新栅格地图数据
    public void updateGridMap(byte[][] mapData, GridMapBuilder.GridMapInfo mapInfo) {
        if (gridMapView != null) {
            gridMapView.updateMapData(mapData, mapInfo);
        }
    }

    public void setTrajectoryVisibility(boolean visible) {
        if (trajectoryView != null) {
            trajectoryView.setVisibility(visible ? View.VISIBLE : View.GONE);
            trajectoryView.invalidate();
        }
    }

    public void invalidateTrajectoryView() {
        if (trajectoryView != null) {
            trajectoryView.invalidate();
        }
    }

    public void invalidateGridMapView() {
        if (gridMapView != null) {
            gridMapView.invalidate();
        }
    }

    public void updateTrajectoryScale(float resolution) {
        if (trajectoryView != null) {
            trajectoryView.updateScale(resolution);
        }
    }

    // ✅ 移除了整个 addRosPublishButton() 方法

    private void addDiagnosticButton() {
        diagButton = new Button(activity);
        diagButton.setText("地图诊断");
        diagButton.setTextSize(12);
        diagButton.setOnClickListener(v -> showDiagnosticDialog());

        ViewGroup rootView = activity.findViewById(android.R.id.content);
        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
        params.gravity = Gravity.BOTTOM | Gravity.END;
        params.setMargins(0, 0, 20, 20);
        rootView.addView(diagButton, params);
    }

    private void showDiagnosticDialog() {
        StringBuilder info = new StringBuilder("系统状态诊断:\n"); // ✅ 改为"系统状态"
        info.append("USB监视器: ").append(activity.getUSBMonitorInstance() != null ? "已初始化" : "未初始化").append("\n");

        UsbManager usbManager = (UsbManager) activity.getSystemService(Context.USB_SERVICE);
        if (usbManager != null) {
            HashMap<String, UsbDevice> deviceList = usbManager.getDeviceList();
            info.append("连接的USB设备数: ").append(deviceList.size()).append("\n");
            for (UsbDevice device : deviceList.values()) {
                info.append("- ").append(device.getDeviceName())
                        .append(" (VID:").append(device.getVendorId())
                        .append(" PID:").append(device.getProductId()).append(")\n");
            }
        }

        info.append("相机对象: ").append(activity.getUVCCamera() != null ? "已创建" : "未创建").append("\n");
        info.append("ROS连接状态: ").append(activity.isRosConnected() ? "已连接" : "未连接").append("\n");
        // ✅ 移除ROS发布状态显示，因为默认就是发布
        info.append("栅格地图状态: ").append(activity.isGridMapInitialized() ? "已初始化" : "未初始化").append("\n");
        info.append("总帧数: ").append(activity.getFrameCount()).append("\n");

        // ✅ 栅格地图诊断信息
        if (activity.isGridMapInitialized()) {
            GridMapBuilder.GridMapInfo mapInfo = activity.getMapInfo();
            if (mapInfo != null) {
                info.append("地图尺寸: ").append(mapInfo.width).append("x").append(mapInfo.height).append("\n");
                info.append("地图分辨率: ").append(String.format("%.3f m/pix", mapInfo.resolution)).append("\n");
                info.append("地图原点: (").append(String.format("%.2f, %.2f", mapInfo.originX, mapInfo.originY)).append(")\n");
            }
        }

        new AlertDialog.Builder(activity)
                .setTitle("系统诊断") // ✅ 改为"系统诊断"
                .setMessage(info.toString())
                .setPositiveButton("确定", null)
                .setNeutralButton("复制信息", (dialog, which) -> {
                    ClipboardManager clipboard = (ClipboardManager) activity.getSystemService(Context.CLIPBOARD_SERVICE);
                    ClipData clip = ClipData.newPlainText("系统诊断", info.toString());
                    clipboard.setPrimaryClip(clip);
                    Toast.makeText(activity, "诊断信息已复制", Toast.LENGTH_SHORT).show();
                })
                .show();
    }

    // ✅ 简化的栅格地图显示类 - 右侧分屏显示
    public class GridMapView extends View {
        private Paint occupiedPaint, freePaint, unknownPaint, textPaint, axisPaint, borderPaint;
        private byte[][] mapData;
        private GridMapBuilder.GridMapInfo mapInfo;
        private Bitmap mapBitmap;
        private Matrix displayMatrix;
        private float displayScale = 5.0f; // 显示缩放
        private float offsetX = 0, offsetY = 0;

        // 触摸控制
        private float lastTouchX, lastTouchY;
        private boolean isDragging = false;

        public GridMapView(Context context) {
            super(context);
            init();
        }

        private void init() {
            setBackgroundColor(Color.argb(200, 0, 0, 0)); // 半透明黑色背景

            occupiedPaint = new Paint();
            occupiedPaint.setColor(Color.BLACK);
            occupiedPaint.setStyle(Paint.Style.FILL);

            freePaint = new Paint();
            freePaint.setColor(Color.WHITE);
            freePaint.setStyle(Paint.Style.FILL);

            unknownPaint = new Paint();
            unknownPaint.setColor(Color.GRAY);
            unknownPaint.setStyle(Paint.Style.FILL);

            textPaint = new Paint();
            textPaint.setColor(Color.YELLOW);
            textPaint.setTextSize(16);
            textPaint.setAntiAlias(true);
            textPaint.setShadowLayer(1, 0, 0, Color.BLACK);

            axisPaint = new Paint();
            axisPaint.setColor(Color.RED);
            axisPaint.setStyle(Paint.Style.STROKE);
            axisPaint.setStrokeWidth(2);
            axisPaint.setAntiAlias(true);

            borderPaint = new Paint();
            borderPaint.setColor(Color.GREEN);
            borderPaint.setStyle(Paint.Style.STROKE);
            borderPaint.setStrokeWidth(2);
            borderPaint.setAntiAlias(true);

            displayMatrix = new Matrix();
        }

        // ✅ 更新地图数据
        public void updateMapData(byte[][] newMapData, GridMapBuilder.GridMapInfo newMapInfo) {
            if (newMapData == null || newMapInfo == null) {
                Log.w(TAG, "地图数据为空");
                return;
            }

            this.mapData = newMapData;
            this.mapInfo = newMapInfo;

            // 生成地图位图
            generateMapBitmap();

            // 刷新视图
            post(this::invalidate);

            Log.d(TAG, String.format("更新栅格地图显示: %dx%d", mapInfo.width, mapInfo.height));
        }

        // ✅ 生成地图位图
        private void generateMapBitmap() {
            if (mapData == null || mapInfo == null) return;

            try {
                mapBitmap = Bitmap.createBitmap(mapInfo.width, mapInfo.height, Bitmap.Config.ARGB_8888);

                for (int y = 0; y < mapInfo.height; y++) {
                    for (int x = 0; x < mapInfo.width; x++) {
                        int occupancyValue = mapData[y][x] & 0xFF;
                        int color;

                        if (occupancyValue >= 65) {
                            color = Color.BLACK; // 占用
                        } else if (occupancyValue <= 25) {
                            color = Color.WHITE; // 自由
                        } else {
                            color = Color.GRAY;  // 未知
                        }

                        mapBitmap.setPixel(x, y, color);
                    }
                }

            } catch (Exception e) {
                Log.e(TAG, "生成地图位图时出错: " + e.getMessage(), e);
            }
        }

        @Override
        protected void onDraw(Canvas canvas) {
            super.onDraw(canvas);

            int width = getWidth();
            int height = getHeight();

            // ✅ 绘制右侧分屏边框
            canvas.drawRect(0, 0, width, height, borderPaint);

            // 绘制地图
            if (mapBitmap != null && mapInfo != null) {
                drawGridMap(canvas, width, height);
            } else {
                // 显示等待地图的消息
                textPaint.setTextSize(18);
                canvas.drawText("等待栅格地图...", width/2 - 100, height/2, textPaint);
            }

            // 绘制信息文本
            drawMapInfo(canvas);
        }

        // ✅ 绘制栅格地图
        private void drawGridMap(Canvas canvas, int width, int height) {
            // 计算显示变换矩阵
            displayMatrix.reset();

            // 居中显示
            float centerX = width / 2f;
            float centerY = height / 2f;

            // 缩放和平移
            displayMatrix.postScale(displayScale, displayScale);
            displayMatrix.postTranslate(centerX - (mapInfo.width * displayScale) / 2f + offsetX,
                    centerY - (mapInfo.height * displayScale) / 2f + offsetY);

            // 绘制地图位图
            Paint bitmapPaint = new Paint();
            bitmapPaint.setFilterBitmap(false); // 保持像素清晰
            canvas.drawBitmap(mapBitmap, displayMatrix, bitmapPaint);

            // 绘制地图边框
            drawMapBorder(canvas, centerX, centerY);

            // 绘制坐标轴
            drawCoordinateSystem(canvas, centerX, centerY);
        }

        // ✅ 绘制地图边框
        private void drawMapBorder(Canvas canvas, float centerX, float centerY) {
            Paint mapBorderPaint = new Paint();
            mapBorderPaint.setColor(Color.CYAN);
            mapBorderPaint.setStyle(Paint.Style.STROKE);
            mapBorderPaint.setStrokeWidth(2);

            float left = centerX - (mapInfo.width * displayScale) / 2f + offsetX;
            float top = centerY - (mapInfo.height * displayScale) / 2f + offsetY;
            float right = left + mapInfo.width * displayScale;
            float bottom = top + mapInfo.height * displayScale;

            canvas.drawRect(left, top, right, bottom, mapBorderPaint);
        }

        // ✅ 绘制坐标系
        private void drawCoordinateSystem(Canvas canvas, float centerX, float centerY) {
            // 计算原点在屏幕上的位置
            float originScreenX = centerX + offsetX - (mapInfo.originX / mapInfo.resolution) * displayScale;
            float originScreenY = centerY + offsetY + (mapInfo.originY / mapInfo.resolution) * displayScale;

            // 绘制坐标轴
            canvas.drawLine(0, originScreenY, getWidth(), originScreenY, axisPaint);
            canvas.drawLine(originScreenX, 0, originScreenX, getHeight(), axisPaint);

            // 标记原点
            canvas.drawCircle(originScreenX, originScreenY, 6, axisPaint);
            canvas.drawText("O", originScreenX + 8, originScreenY - 8, textPaint);
        }

        // ✅ 绘制地图信息
        private void drawMapInfo(Canvas canvas) {
            float y = 25;
            textPaint.setTextSize(34);

            if (mapInfo != null) {
                canvas.drawText("栅格地图", 10, y, textPaint);
                y += 40;
                canvas.drawText(mapInfo.width + "x" + mapInfo.height, 10, y, textPaint);
                y += 40;
                canvas.drawText("分辨率: " + String.format("%.3f m/pix", mapInfo.resolution), 10, y, textPaint);
                y += 40;
                canvas.drawText("缩放: " + String.format("%.1fx", displayScale), 10, y, textPaint);
                y += 40;
                canvas.drawText("原点: (" + String.format("%.1f, %.1f", mapInfo.originX, mapInfo.originY) + ")", 10, y, textPaint);
            } else {
                canvas.drawText("栅格地图", 10, y, textPaint);
                y += 40;
                canvas.drawText("等待数据...", 10, y, textPaint);
            }

            // 显示操作提示
            textPaint.setTextSize(22);
            canvas.drawText("拖拽平移地图", 10, getHeight() - 30, textPaint);
        }

        // ✅ 触摸控制
        @Override
        public boolean onTouchEvent(MotionEvent event) {
            switch (event.getAction()) {
                case MotionEvent.ACTION_DOWN:
                    lastTouchX = event.getX();
                    lastTouchY = event.getY();
                    isDragging = false;
                    return true;

                case MotionEvent.ACTION_MOVE:
                    float deltaX = event.getX() - lastTouchX;
                    float deltaY = event.getY() - lastTouchY;

                    offsetX += deltaX;
                    offsetY += deltaY;

                    lastTouchX = event.getX();
                    lastTouchY = event.getY();
                    isDragging = true;

                    invalidate();
                    return true;

                case MotionEvent.ACTION_UP:
                    return true;
            }

            return super.onTouchEvent(event);
        }
    }

    // 轨迹显示的内部类（保持原有功能）
    public class TrajectoryView extends View {
        private Paint pathPaint, currentPosePaint, textPaint, axisPaint, gridPaint;
        private float gridInterval = 1.0f;
        private final int GRID_COUNT = 10;
        private float scale = 200.0f;

        public TrajectoryView(Context context) {
            super(context);
            init();
        }

        public void updateScale(float mapResolution) {
            if (mapResolution > 0) {
                this.scale = 200.0f / mapResolution;
                invalidate();
            }
        }

        private void init() {
            setBackgroundColor(Color.argb(80, 0, 0, 0));

            pathPaint = new Paint();
            pathPaint.setColor(Color.GREEN);
            pathPaint.setStyle(Paint.Style.STROKE);
            pathPaint.setStrokeWidth(4);
            pathPaint.setAntiAlias(true);

            currentPosePaint = new Paint();
            currentPosePaint.setColor(Color.RED);
            currentPosePaint.setStyle(Paint.Style.FILL);
            currentPosePaint.setAntiAlias(true);

            textPaint = new Paint();
            textPaint.setColor(Color.WHITE);
            textPaint.setTextSize(24);
            textPaint.setAntiAlias(true);
            textPaint.setShadowLayer(2, 0, 0, Color.BLACK);

            axisPaint = new Paint();
            axisPaint.setColor(Color.WHITE);
            axisPaint.setStyle(Paint.Style.STROKE);
            axisPaint.setStrokeWidth(2);
            axisPaint.setAntiAlias(true);

            gridPaint = new Paint();
            gridPaint.setColor(Color.argb(100, 200, 200, 200));
            gridPaint.setStyle(Paint.Style.STROKE);
            gridPaint.setStrokeWidth(1);
            gridPaint.setAntiAlias(true);
        }

        @Override
        protected void onDraw(Canvas canvas) {
            super.onDraw(canvas);

            int width = getWidth();
            int height = getHeight();
            int centerX = width / 2;
            int centerY = height / 2;

            drawGridAndAxis(canvas, centerX, centerY);

            List<double[]> trajectoryPoints = activity.getTrajectoryPoints();
            if (trajectoryPoints.isEmpty()) return;

            Path path = new Path();
            boolean first = true;

            for (double[] point : trajectoryPoints) {
                float x = centerX + (float)(point[0] * scale);
                float y = centerY - (float)(point[1] * scale);

                if (first) {
                    path.moveTo(x, y);
                    first = false;
                } else {
                    path.lineTo(x, y);
                }
            }
            canvas.drawPath(path, pathPaint);
        }

        private void drawGridAndAxis(Canvas canvas, int centerX, int centerY) {
            int width = getWidth();
            int height = getHeight();

            canvas.drawLine(0, centerY, width, centerY, axisPaint);
            canvas.drawLine(centerX, 0, centerX, height, axisPaint);

            for (int i = -GRID_COUNT; i <= GRID_COUNT; i++) {
                if (i == 0) continue;

                float pos = i * gridInterval * scale;

                canvas.drawLine(0, centerY - pos, width, centerY - pos, gridPaint);
                canvas.drawLine(centerX + pos, 0, centerX + pos, height, gridPaint);

                String valueX = String.format("%.1f", i * gridInterval);
                String valueY = String.format("%.1f", i * gridInterval);


                canvas.drawText(valueX, centerX + pos - 15, centerY + 40, textPaint);
                canvas.drawText(valueY, centerX + 10, centerY - pos + 10, textPaint);
            }

            canvas.drawText("O", centerX - 30, centerY + 30, textPaint);
            canvas.drawText("X", width - 40, centerY + 30, textPaint);
            canvas.drawText("Y", centerX + 10, 40, textPaint);
        }

    }
}