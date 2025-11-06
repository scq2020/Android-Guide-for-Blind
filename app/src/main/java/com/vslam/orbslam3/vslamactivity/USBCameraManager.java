package com.vslam.orbslam3.camera;

import android.content.Context;
import android.graphics.SurfaceTexture;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.view.Surface;
import android.widget.Button;
import android.widget.Toast;

import com.serenegiant.usb.*;
import com.serenegiant.usb.USBMonitor.OnDeviceConnectListener;
import com.serenegiant.usb.USBMonitor.UsbControlBlock;

import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class USBCameraManager {
    private static final String TAG = "USBCameraManager";
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 480;

    private final Context context;
    private final USBMonitor usbMonitor;
    private UVCCamera uvcCamera;
    private Surface previewSurface;
    private final Object cameraLock = new Object();
    private final ExecutorService executor = Executors.newSingleThreadExecutor();

    private CameraCallback callback;
    private boolean isActive = false;
    private boolean isPreview = false;
    private boolean cameraDialogShowing = false;


    public interface CameraCallback {
        void onCameraConnected(UVCCamera camera);
        void onCameraDisconnected();
        void onCameraAttach(UsbDevice device);
        void onCameraDetach(UsbDevice device);
        void onConnectFail(String reason);
        void onFrameReceived(ByteBuffer frame);
        void onShowCameraDialog(); // 添加显示对话框的回调
        void runOnUIThread(Runnable task);
        void runInBackground(Runnable task);
    }

    public USBCameraManager(Context context, CameraCallback callback) {
        this.context = context.getApplicationContext();
        this.callback = callback;
        this.usbMonitor = new USBMonitor(this.context, deviceConnectListener);
    }



    public void registerUSBMonitor() {
        if (usbMonitor != null) {
            usbMonitor.register();
        }
    }

    public void unregisterUSBMonitor() {
        if (usbMonitor != null) {
            usbMonitor.unregister();
        }
    }

    /**
     * 处理相机连接/断开
     */
    public void handleCameraConnection() {
        UsbManager usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
        if (usbManager != null) {
            HashMap<String, UsbDevice> deviceList = usbManager.getDeviceList();
            if (deviceList.isEmpty()) {
                toast("未检测到USB设备，请先连接USB相机");
                if (callback != null) callback.onConnectFail("No USB device found");
                return;
            }
        }

        if (uvcCamera == null) {
            // 重置状态并显示对话框
            cameraDialogShowing = false;
            showCameraDialogSafely();
        } else {
            disconnectCamera();
        }
    }

    private void showCameraDialogSafely() {
        if (!cameraDialogShowing && callback != null) {
            cameraDialogShowing = true;
            callback.onShowCameraDialog(); // 通过回调让Activity显示对话框
        }
    }

    public void onDialogResult(boolean canceled) {
        cameraDialogShowing = false;
        if (canceled) {
            toast("USB相机连接已取消");
        }
    }

    public void disconnectCamera() {
        executor.execute(() -> {
            synchronized (cameraLock) {
                if (uvcCamera != null) {
                    try {
                        if (isPreview) {
                            uvcCamera.stopPreview();
                        }
                        uvcCamera.close();
                        uvcCamera.destroy();
                    } catch (Exception e) {
                        // 忽略断开连接时的异常
                    }
                    uvcCamera = null;
                    isActive = false;
                    isPreview = false;
                }
                if (previewSurface != null) {
                    previewSurface.release();
                    previewSurface = null;
                }
            }

            // 重置对话框状态
            cameraDialogShowing = false;

            if (callback != null) {
                callback.runOnUIThread(() -> callback.onCameraDisconnected());
            }
        });
    }

    public void setPreviewSurface(Surface surface) {
        this.previewSurface = surface;
    }

    private final OnDeviceConnectListener deviceConnectListener = new OnDeviceConnectListener() {
        @Override
        public void onAttach(final UsbDevice device) {
            // 可以调用回调或者只是空实现
            if (callback != null) callback.onCameraAttach(device);
        }

        @Override
        public void onConnect(final UsbDevice device, final UsbControlBlock ctrlBlock, final boolean createNew) {
            toast("正在连接USB相机...");
            executor.execute(() -> {
                synchronized (cameraLock) {
                    if (uvcCamera != null) {
                        uvcCamera.destroy();
                    }
                    isActive = false;
                    isPreview = false;
                }

                final UVCCamera camera = new UVCCamera();
                try {
                    camera.open(ctrlBlock);

                    // 尝试不同的预览格式
                    try {
                        camera.setPreviewSize(CAMERA_WIDTH, CAMERA_HEIGHT, UVCCamera.FRAME_FORMAT_MJPEG);
                    } catch (Exception e1) {
                        try {
                            camera.setPreviewSize(CAMERA_WIDTH, CAMERA_HEIGHT, UVCCamera.FRAME_FORMAT_YUYV);
                        } catch (Exception e2) {
                            try {
                                camera.setPreviewSize(CAMERA_WIDTH, CAMERA_HEIGHT, UVCCamera.DEFAULT_PREVIEW_MODE);
                            } catch (Exception e3) {
                                camera.destroy();

                                if (callback != null) callback.onConnectFail("Failed to set preview size");
                                return;
                            }
                        }
                    }

                    // 设置帧回调
                    camera.setFrameCallback(frameCallback, UVCCamera.PIXEL_FORMAT_YUV420SP);

                    if (callback != null) {
                        callback.runOnUIThread(() -> {
                            synchronized (cameraLock) {
                                try {
                                    if (previewSurface != null && previewSurface.isValid()) {
                                        // 创建虚拟显示表面
                                        SurfaceTexture dummyTexture = new SurfaceTexture(0);
                                        Surface dummySurface = new Surface(dummyTexture);
                                        camera.setPreviewDisplay(dummySurface);
                                        camera.startPreview();

                                        uvcCamera = camera;
                                        isActive = true;
                                        isPreview = false;


                                        callback.onCameraConnected(camera);
                                    } else {
                                        camera.destroy();

                                        callback.onConnectFail("Invalid preview surface");
                                    }
                                } catch (Exception e) {
                                    camera.destroy();

                                    callback.onConnectFail("Failed to start preview: " + e.getMessage());
                                }
                            }
                        });
                    }
                } catch (Exception e) {
                    camera.destroy();

                    if (callback != null) callback.onConnectFail("Failed to open camera: " + e.getMessage());
                }
            });
        }

        @Override
        public void onDisconnect(final UsbDevice device, final UsbControlBlock ctrlBlock) {
            executor.execute(() -> {
                synchronized (cameraLock) {
                    if (uvcCamera != null) {
                        uvcCamera.close();
                        if (previewSurface != null) {
                            previewSurface.release();
                            previewSurface = null;
                        }
                        isActive = false;
                        isPreview = false;
                    }
                }
                if (callback != null) {
                    callback.runOnUIThread(() -> {
                        callback.onCameraDisconnected();

                        toast("USB相机已断开连接");
                    });
                }
            });
        }

        @Override
        public void onDettach(final UsbDevice device) {
            toast("USB设备已移除");

            if (callback != null) callback.onCameraDetach(device);
        }

        @Override
        public void onCancel(final UsbDevice device) {
            cameraDialogShowing = false;

            if (callback != null) {
                callback.runOnUIThread(() -> toast("相机连接已取消"));
            }
        }
    };

    private final IFrameCallback frameCallback = new IFrameCallback() {
        @Override
        public void onFrame(final ByteBuffer frame) {
            if (callback != null) {
                callback.onFrameReceived(frame);
            }
        }
    };

    private void toast(String msg) {
        if (callback != null) {
            callback.runOnUIThread(() -> Toast.makeText(context, msg, Toast.LENGTH_SHORT).show());
        }
    }

    public void destroy() {
        disconnectCamera();
        if (usbMonitor != null) {
            usbMonitor.destroy();
        }
        executor.shutdown();
    }

    // Getter methods
    public USBMonitor getUSBMonitor() { return usbMonitor; }
    public UVCCamera getUVCCamera() { return uvcCamera; }
    public boolean isActive() { return isActive; }

}