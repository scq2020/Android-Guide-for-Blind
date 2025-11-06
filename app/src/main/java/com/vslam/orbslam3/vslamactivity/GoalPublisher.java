// GoalPublisher.java
package com.vslam.orbslam3.vslamactivity;

import android.content.Context;
import android.graphics.Color;
import android.util.Log;
import android.util.TypedValue;
import android.view.Gravity;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.Toast;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import std_msgs.Int32;

/**
 * 目标发布器类 - 处理导航目标的发布
 * 独立管理目标发布器，不依赖RosConnection修改
 */
public class GoalPublisher {
    private static final String TAG = "GoalPublisher";

    // 目标ID常量
    public static final int GOAL_CHAIR = 56;
    public static final int GOAL_CUP = 41;
    public static final int GOAL_TABLE = 60;

    private final Context context;
    private final VslamActivity activity;
    private RosManager rosManager;

    // ROS发布器
    private Publisher<std_msgs.Int32> goalPublisher;
    private ConnectedNode connectedNode;

    // UI组件
    private Button chairButton;
    private Button cupButton;
    private Button tableButton;
    private LinearLayout buttonContainer;

    // 状态变量
    private boolean isInitialized = false;
    private int lastPublishedGoal = -1;

    public GoalPublisher(VslamActivity activity, RosManager rosManager) {
        this.activity = activity;
        this.context = activity;
        this.rosManager = rosManager;

        setupUI();
        initializeRosPublisher();
    }

    /**
     * 初始化ROS发布器
     */
    private void initializeRosPublisher() {
        // 延迟初始化，等待ROS连接
        new Thread(() -> {
            int retryCount = 0;
            while (!isInitialized && retryCount < 30) { // 最多等待30秒
                try {
                    Thread.sleep(1000);
                    if (rosManager != null && rosManager.isConnected()) {
                        // 获取ConnectedNode
                        connectedNode = getConnectedNodeFromRosManager();
                        if (connectedNode != null) {
                            createGoalPublisher();
                            break;
                        }
                    }
                    retryCount++;
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }

            if (!isInitialized) {
                Log.w(TAG, "目标发布器初始化超时");
                //showToast("目标发布器初始化失败");
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
     * 创建目标发布器
     */
    private void createGoalPublisher() {
        try {
            if (connectedNode != null) {
                goalPublisher = connectedNode.newPublisher("/goal_no", std_msgs.Int32._TYPE);
                isInitialized = true;

                Log.i(TAG, "目标发布器初始化成功");
                activity.runOnUiThread(() -> {
                    //showToast("目标发布器已就绪");
                    updateButtonStates(-1); // 启用所有按钮
                });
            }
        } catch (Exception e) {
            Log.e(TAG, "创建目标发布器失败: " + e.getMessage(), e);
            //showToast("创建目标发布器失败");
        }
    }

    /**
     * 设置UI界面
     */
    private void setupUI() {
        // 创建按钮容器
        buttonContainer = new LinearLayout(context);
        buttonContainer.setOrientation(LinearLayout.HORIZONTAL);
        buttonContainer.setBackgroundColor(Color.argb(128, 0, 0, 0));
        buttonContainer.setPadding(8, 8, 8, 8); // 减小容器内边距
        buttonContainer.setGravity(Gravity.CENTER); // 按钮居中显示

        // 创建椅子按钮
        chairButton = createGoalButton("瓶子", Color.parseColor("#FF6B6B"), GOAL_CUP);

        // 创建茶杯按钮
        cupButton = createGoalButton("椅子", Color.parseColor("#4ECDC4"), GOAL_CHAIR);

        // 创建桌子按钮
        tableButton = createGoalButton("桌子", Color.parseColor("#45B7D1"), GOAL_TABLE);

        // 添加按钮到容器
        buttonContainer.addView(chairButton);
        buttonContainer.addView(cupButton);
        buttonContainer.addView(tableButton);

        // 添加到主布局
        addToMainLayout();

        // 初始状态：禁用按钮直到ROS连接
        //setEnabled(false);
    }

    /**
     * 创建目标按钮 - 1/5宽度版本
     */
    private Button createGoalButton(String text, int color, int goalId) {
        Button button = new Button(context);
        button.setText(text);
        button.setTextColor(Color.WHITE);
        button.setBackgroundColor(color);
        button.setTextSize(TypedValue.COMPLEX_UNIT_SP, 10); // 减小字体大小：14 -> 10
        button.setPadding(8, 5, 8, 5); // 减小内边距：20,10,20,10 -> 8,5,8,5

        // 设置固定宽度而不是权重分配
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(
                dpToPx(80), ViewGroup.LayoutParams.WRAP_CONTENT); // 固定宽度80dp
        params.setMargins(3, 0, 3, 0); // 减小间距：5 -> 3
        button.setLayoutParams(params);

        // 设置点击事件
        //button.setOnClickListener(v -> publishGoal(goalId));
        button.setOnClickListener(v -> {
            // 先执行路径规划
            if (activity != null) {
                activity.planPathToObject(goalId);
            }
            // 再尝试ROS发布
            publishGoal(goalId);
        });
        return button;
    }

    /**
     * dp转px的辅助方法
     */
    private int dpToPx(int dp) {
        return (int) TypedValue.applyDimension(
                TypedValue.COMPLEX_UNIT_DIP, dp,
                context.getResources().getDisplayMetrics());
    }

    /**
     * 添加到主布局
     */
    private void addToMainLayout() {
        ViewGroup rootView = activity.findViewById(android.R.id.content);

        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT, // 改为自适应宽度
                ViewGroup.LayoutParams.WRAP_CONTENT);
        params.gravity = Gravity.BOTTOM | Gravity.CENTER_HORIZONTAL;
        params.setMargins(20, 0, 20, 100);

        rootView.addView(buttonContainer, params);
    }

    /**
     * 发布目标ID到/goal_no话题
     */
    public void publishGoal(int goalId) {
        if (!isInitialized || goalPublisher == null) {
            Log.w(TAG, "目标发布器未初始化");
            //showToast("目标发布器未就绪");
            return;
        }

        if (connectedNode == null) {
            Log.w(TAG, "ROS节点未连接");
            //showToast("ROS未连接");
            return;
        }

        try {
            // 创建并发布消息
            std_msgs.Int32 goalMsg = goalPublisher.newMessage();
            goalMsg.setData(goalId);
            goalPublisher.publish(goalMsg);

            // 记录和显示
            lastPublishedGoal = goalId;
            String goalName = getGoalName(goalId);
            Log.i(TAG, "发布目标: " + goalName + " (ID: " + goalId + ")");
            //showToast("导航目标: " + goalName);

            // 更新按钮状态
            updateButtonStates(goalId);

        } catch (Exception e) {
            Log.e(TAG, "发布目标失败: " + e.getMessage(), e);
            //showToast("发布目标失败: " + e.getMessage());
        }
    }

    /**
     * 更新按钮状态（高亮当前选中的目标）
     */
    private void updateButtonStates(int selectedGoalId) {
        activity.runOnUiThread(() -> {
            if (chairButton == null || cupButton == null || tableButton == null) {
                return;
            }

            // 启用状态
            chairButton.setEnabled(true);
            cupButton.setEnabled(true);
            tableButton.setEnabled(true);

            if (selectedGoalId == -1) {
                // 重置状态
                chairButton.setAlpha(1.0f);
                cupButton.setAlpha(1.0f);
                tableButton.setAlpha(1.0f);

                chairButton.setScaleX(1.0f);
                chairButton.setScaleY(1.0f);
                cupButton.setScaleX(1.0f);
                cupButton.setScaleY(1.0f);
                tableButton.setScaleX(1.0f);
                tableButton.setScaleY(1.0f);
            } else {
                // 高亮选中的按钮
                chairButton.setAlpha(selectedGoalId == GOAL_CHAIR ? 1.0f : 0.7f);
                cupButton.setAlpha(selectedGoalId == GOAL_CUP ? 1.0f : 0.7f);
                tableButton.setAlpha(selectedGoalId == GOAL_TABLE ? 1.0f : 0.7f);

                // 添加选中效果
                chairButton.setScaleX(selectedGoalId == GOAL_CHAIR ? 1.1f : 1.0f);
                chairButton.setScaleY(selectedGoalId == GOAL_CHAIR ? 1.1f : 1.0f);

                cupButton.setScaleX(selectedGoalId == GOAL_CUP ? 1.1f : 1.0f);
                cupButton.setScaleY(selectedGoalId == GOAL_CUP ? 1.1f : 1.0f);

                tableButton.setScaleX(selectedGoalId == GOAL_TABLE ? 1.1f : 1.0f);
                tableButton.setScaleY(selectedGoalId == GOAL_TABLE ? 1.1f : 1.0f);
            }
        });
    }

    /**
     * 根据ID获取目标名称
     */
    private String getGoalName(int goalId) {
        switch (goalId) {
            case GOAL_CHAIR: return "椅子";
            case GOAL_CUP: return "瓶子";
            case GOAL_TABLE: return "桌子";
            default: return "未知目标";
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
     * 设置按钮可见性
     */
    public void setVisible(boolean visible) {
        if (buttonContainer != null) {
            activity.runOnUiThread(() ->
                    buttonContainer.setVisibility(visible ?
                            android.view.View.VISIBLE : android.view.View.GONE));
        }
    }

    /**
     * 设置按钮启用状态
     */
    public void setEnabled(boolean enabled) {
        activity.runOnUiThread(() -> {
            if (chairButton != null) chairButton.setEnabled(enabled);
            if (cupButton != null) cupButton.setEnabled(enabled);
            if (tableButton != null) tableButton.setEnabled(enabled);
        });
    }

    /**
     * 检查目标发布器是否就绪
     */
    public boolean isReady() {
        return isInitialized && goalPublisher != null && connectedNode != null;
    }

    /**
     * 获取最后发布的目标
     */
    public int getLastPublishedGoal() {
        return lastPublishedGoal;
    }

    /**
     * 手动重试初始化
     */
    public void retryInitialization() {
        if (!isInitialized) {
            Log.i(TAG, "手动重试初始化目标发布器");
            initializeRosPublisher();
        }
    }

    /**
     * 获取目标发布器状态信息
     */
    public String getStatusInfo() {
        StringBuilder status = new StringBuilder();
        status.append("目标发布器状态:\n");
        status.append("初始化: ").append(isInitialized ? "✅" : "❌").append("\n");
        status.append("ROS连接: ").append(connectedNode != null ? "✅" : "❌").append("\n");
        status.append("发布器: ").append(goalPublisher != null ? "✅" : "❌").append("\n");
        status.append("最后发布: ").append(lastPublishedGoal == -1 ? "无" : getGoalName(lastPublishedGoal));
        return status.toString();
    }

    /**
     * 清理资源
     */
    public void shutdown() {
        Log.i(TAG, "正在关闭目标发布器...");

        isInitialized = false;

        if (goalPublisher != null) {
            try {
                // ROS发布器会在节点关闭时自动清理
                goalPublisher = null;
                Log.i(TAG, "目标发布器已清理");
            } catch (Exception e) {
                Log.e(TAG, "清理目标发布器时出错: " + e.getMessage(), e);
            }
        }

        if (buttonContainer != null && buttonContainer.getParent() != null) {
            ((ViewGroup) buttonContainer.getParent()).removeView(buttonContainer);
            Log.i(TAG, "目标发布器UI已移除");
        }

        chairButton = null;
        cupButton = null;
        tableButton = null;
        buttonContainer = null;
        rosManager = null;
        connectedNode = null;

        Log.i(TAG, "目标发布器关闭完成");
    }
}