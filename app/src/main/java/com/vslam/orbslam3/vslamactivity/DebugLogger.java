package com.vslam.orbslam3.vslamactivity;

import android.content.ClipData;
import android.content.ClipboardManager;
import android.content.Context;
import android.util.Log;
import android.view.View;
import android.widget.TextView;

/**
 * DebugLogger 类用于处理调试信息的记录和显示
 */
public class DebugLogger {

    private TextView debugTextView;
    private final Context context;
    private StringBuilder debugMessages; // 存储所有调试信息

    public DebugLogger(Context context) {
        this.context = context;
        this.debugMessages = new StringBuilder();

        // 初始化 TextView
        initDebugTextView();
    }

    // 初始化 TextView
    private void initDebugTextView() {
        if (context instanceof VslamActivity) {
            debugTextView = ((VslamActivity) context).findViewById(R.id.debugTextView);
            // 初始时隐藏TextView，避免显示空的黑框
            if (debugTextView != null) {
                debugTextView.setVisibility(View.GONE);
            }
        }
    }

    // 更新调试信息到 TextView
    public void updateDebugTextView(final String message) {
        if (debugTextView != null && message != null && !message.trim().isEmpty()) {
            ((VslamActivity) context).runOnUiThread(() -> {
                // 如果是第一次添加信息，显示TextView
                if (debugMessages.length() == 0) {
                    debugTextView.setVisibility(View.VISIBLE);
                }

                debugMessages.append(message).append("\n"); // 将信息添加到 StringBuilder
                debugTextView.append(message); // 先添加信息
                debugTextView.append("\n"); // 然后添加换行
                copyToClipboard(debugMessages.toString()); // 每次更新时复制所有调试信息
            });
        }
    }

    // 清空调试信息
    public void clearDebugInfo() {
        if (debugTextView != null) {
            ((VslamActivity) context).runOnUiThread(() -> {
                debugMessages.setLength(0); // 清空StringBuilder
                debugTextView.setText(""); // 清空TextView内容
                debugTextView.setVisibility(View.GONE); // 隐藏TextView
            });
        }
    }

    // 检查是否有调试信息
    public boolean hasDebugInfo() {
        return debugMessages.length() > 0;
    }

    // 获取调试信息
    public String getDebugInfo() {
        return debugMessages.toString();
    }

    // 将调试信息复制到剪贴板
    private void copyToClipboard(String messages) {
        ClipboardManager clipboard = (ClipboardManager) context.getSystemService(Context.CLIPBOARD_SERVICE);
        ClipData clip = ClipData.newPlainText("Debug Info", messages);
        clipboard.setPrimaryClip(clip);
    }
}