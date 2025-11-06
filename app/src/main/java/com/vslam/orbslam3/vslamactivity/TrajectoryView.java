package com.vslam.orbslam3.vslamactivity;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.view.View;

// 添加缺失的导入
import java.util.List;
import java.util.ArrayList;

/**
 * 轨迹显示视图，用于可视化SLAM系统的相机轨迹
 */
public class TrajectoryView extends View {
    private Paint trajectoryPaint, currentPosPaint, gridPaint, textPaint;
    private float centerX, centerY;
    private float scale;
    private static final double TRAJECTORY_SCALE = 200.0;

    // 轨迹点数据列表，由VslamActivity管理和更新
    private List<double[]> trajectoryPoints;
    private boolean showTrajectory = true;

    public TrajectoryView(Context context) {
        super(context);
        init();
        scale = (float)TRAJECTORY_SCALE;
        trajectoryPoints = new ArrayList<>();
    }

    /**
     * 设置要显示的轨迹点
     * @param points 轨迹点列表
     */
    public void setTrajectoryPoints(List<double[]> points) {
        this.trajectoryPoints = points;
        invalidate();
    }

    /**
     * 设置是否显示轨迹
     * @param show 显示轨迹的标志
     */
    public void setShowTrajectory(boolean show) {
        this.showTrajectory = show;
        invalidate();
    }

    private void init() {
        trajectoryPaint = new Paint();
        trajectoryPaint.setColor(Color.RED);
        trajectoryPaint.setStrokeWidth(4);
        trajectoryPaint.setStyle(Paint.Style.STROKE);
        trajectoryPaint.setAntiAlias(true);

        currentPosPaint = new Paint();
        currentPosPaint.setColor(Color.BLUE);
        currentPosPaint.setStyle(Paint.Style.FILL);
        currentPosPaint.setAntiAlias(true);

        gridPaint = new Paint();
        gridPaint.setColor(Color.argb(100, 200, 200, 200));
        gridPaint.setStrokeWidth(1);
        gridPaint.setStyle(Paint.Style.STROKE);

        textPaint = new Paint();
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(30);
        textPaint.setAntiAlias(true);

        setBackgroundColor(Color.argb(100, 0, 0, 0));
    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        centerX = w / 2f;
        centerY = h / 2f;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        if (!showTrajectory) return;

        drawGrid(canvas);

        if (trajectoryPoints != null && trajectoryPoints.size() >= 2) {
            Path trajectoryPath = new Path();
            boolean pathStarted = false;

            for (double[] point : trajectoryPoints) {
                float screenX = centerX + (float)(point[0] * scale);
                float screenY = centerY - (float)(point[1] * scale);

                if (!pathStarted) {
                    trajectoryPath.moveTo(screenX, screenY);
                    pathStarted = true;
                } else {
                    trajectoryPath.lineTo(screenX, screenY);
                }
            }

            canvas.drawPath(trajectoryPath, trajectoryPaint);

            double[] currentPos = trajectoryPoints.get(trajectoryPoints.size() - 1);
            float currentX = centerX + (float)(currentPos[0] * scale);
            float currentY = centerY - (float)(currentPos[1] * scale);
            canvas.drawCircle(currentX, currentY, 10, currentPosPaint);

            String posText = String.format("X: %.2f, Y: %.2f, Z: %.2f",
                    currentPos[0], currentPos[1], currentPos.length > 2 ? currentPos[2] : 0);
            canvas.drawText(posText, 20, getHeight() - 20, textPaint);
        }

        canvas.drawCircle(centerX, centerY, 5, gridPaint);
        canvas.drawText("O", centerX + 10, centerY - 10, textPaint);
    }

    private void drawGrid(Canvas canvas) {
        int gridSize = 50;
        int gridCount = 10;

        for (int i = -gridCount; i <= gridCount; i++) {
            float y = centerY - i * gridSize;
            canvas.drawLine(0, y, getWidth(), y, gridPaint);

            float x = centerX + i * gridSize;
            canvas.drawLine(x, 0, x, getHeight(), gridPaint);

            if (i != 0 && i % 2 == 0) {
                canvas.drawText(String.valueOf(i), x, centerY + 30, textPaint);
                canvas.drawText(String.valueOf(i), centerX + 10, y, textPaint);
            }
        }

        Paint axisPaint = new Paint(gridPaint);
        axisPaint.setStrokeWidth(3);
        axisPaint.setColor(Color.WHITE);

        canvas.drawLine(0, centerY, getWidth(), centerY, axisPaint);
        canvas.drawLine(centerX, 0, centerX, getHeight(), axisPaint);
    }

    /**
     * 清除所有轨迹点
     */
    public void clearTrajectory() {
        if (trajectoryPoints != null) {
            trajectoryPoints.clear();
            invalidate();
        }
    }

    /**
     * 设置轨迹缩放比例
     * @param scale 缩放比例
     */
    public void setTrajectoryScale(float scale) {
        this.scale = scale;
        invalidate();
    }
}