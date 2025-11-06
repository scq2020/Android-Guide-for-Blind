package com.vslam.orbslam3.vslamactivity;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.*;
import android.content.pm.PackageManager;
import android.graphics.*;
import android.hardware.usb.*;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.*;
import android.widget.*;

import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.serenegiant.usb.*;
import com.vslam.orbslam3.camera.USBCameraManager;

import org.opencv.android.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.nio.ByteBuffer;
import java.util.*;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import android.os.Vibrator;
import android.util.Log;
public class VslamActivity extends Activity implements SurfaceHolder.Callback,
        CameraDialog.CameraDialogParent, USBCameraManager.CameraCallback {

    // ==================== 常量声明 ====================
    private static final String TAG = "VslamActivity";
    private static final int PERMISSION_REQUEST_CODE = 100;
    private static final int CAMERA_WIDTH = 1280;
    private static final int CAMERA_HEIGHT = 480;
    private static final int TRAJECTORY_SAMPLING = 5;
    private static final int MAX_TRAJECTORY_POINTS = 500;
    private static final int ROS_PUBLISH_INTERVAL = 100;

    // ==================== 新增：目标对象相关 ====================
    private static final int TARGET_OBJECT_ID = 56; // 目标对象ID
    private String objectsFilePath;

    // ==================== Native方法声明 ====================
    private native boolean isNewKeyFrame();
    private native float[] getCurrentKeyFrameData();
    public native float[] getLocalPointCloud(long leftMatAddr);
    private native int getMapCount();
    private native float[] processStereoFrame(long leftMatAddr, long rightMatAddr, double timestamp);
    public native boolean convertVocabulary(String inputPath, String outputPath);
    public native String testOpenMP();
    public native boolean initializeSLAM(String vocPath, String configPath);
    public native boolean shutdownSLAM(String trajectoryPath);
    private native float[] getTrackedMapPoints();
    private native float[] getAllKeyframeData();
    private native boolean checkLoopClosure();
    public native boolean saveMapOSA(String mapPath);
    private native long generateDepthMapFromSLAM(long leftMatAddr);
    private native void releaseDepthMap(long depthMatAddr);

    // ==================== 成员变量声明 ====================
    // UI组件
    private TextView debugTextView, myTextView, mapCountTextView;
    private SurfaceView mUVCCameraView;
    private CheckBox showTrajectoryCheckBox;
    private Button clearTrajectoryButton;

    // 状态变量
    private static long count = 0;
    private volatile boolean slamInitialized = false;
    private boolean permissionsGranted = false;
    private boolean showTrajectory = false;
    private boolean mIsRendering = false;
    private boolean isRosPublishing = false;
    private boolean mGridMapInitialized = false;
    private boolean mapLoaded = false;
    private boolean mapLoadAttempted = false;
    private boolean lastKeyFrameState = false;
    private boolean useROS = true;

    // 摄像头管理
    private USBCameraManager cameraManager;
    private Surface mPreviewSurface;
    private Mat mCurrentFrame;

    // 位图处理
    private Bitmap mLatestBitmap = null;
    private final Object mBitmapLock = new Object();
    private Handler mRenderHandler;

    // 轨迹和数据
    private List<double[]> trajectoryPoints = new ArrayList<>();
    private float[] lastCameraPose = null;
    private float[] lastMapPoints = null;
    private final Object rosDataLock = new Object();
    private long rosFrameCount = 0;

    // 文件路径
    private String vocPath, configPath, trajectoryPath, mapYamlPath;

    // 管理器和组件
    private final ExecutorService executor = Executors.newSingleThreadExecutor();
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private RosManager rosManager;
    private GridMapManager gridMapManager;
    private Handler rosPublishHandler;
    public VslamUIHelper uiHelper;
    private MapServerManager mapServerManager;
    private GoalPublisher goalPublisher;

    // 性能监控
    private long lastFrameTime = 0;
    private long frameCount = 0;
    private float averageFPS = 0.0f;

    private LinearLayout mapDisplayContainer;
    private ImageView mapImageView;
    private PathPlanningManager pathPlanningManager;
    private float lastSpokenAngle = 0f;
    private double lastSpokenDist = 0.0;
    private boolean firstSpeak = true;

    // ==================== 成员变量声明 ====================
    // 路径规划相关
    private Handler pathUpdateHandler;
    private Runnable pathUpdateRunnable;
    private boolean isPathPlanningActive = false;
    private int currentTargetObjectId = -1;
    private double[] lastPlanningPosition = null;
    private final double REPLAN_THRESHOLD = 0.1;   // 路径规划重新规划阈值10 cm

    private String currentConfigPath; // 当前使用的配置文件路径
    private final Object slamLock = new Object();
    private volatile boolean isRestarting = false;
    private TextView modeStatusTextView; // 显示当前模式
    private TextView headingAngleTextView;
    private HeadingTTS headingTTS;
    private long lastTriggerMillis = 0;

    private volatile boolean isNavigationMode = false;

    private boolean hasReachedTarget = false;  // 是否已到达目标
    private static final double TARGET_REACH_DISTANCE = 0.5;  // 到达目标的距离阈值（米）
    private int previousMapCount = -1;  // 上一次的地图数，用于检测状态变化
    private boolean hasPlayedLocationSuccessAudio = false;  // 是否已播报定位成功语音
    private Vibrator vibrator;  // 震动器

    // ==================== 新增：对象数据类 ====================
    /**
     * 对象数据类
     */
    public static class ObjectData {
        public int id;
        public String name;
        public double x;
        public double y;
        public double z;

        public ObjectData(int id, String name, double x, double y, double z) {
            this.id = id;
            this.name = name;
            this.x = x;
            this.y = y;
            this.z = z;
        }

        @Override
        public String toString() {
            return String.format("ID:%d, Name:%s, Position:(%.3f,%.3f,%.3f)",
                    id, name, x, y, z);
        }
    }

    // 广播接收器
    private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            // USB设备状态变化处理
        }
    };

    // OpenCV回调
    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            if (status == LoaderCallbackInterface.SUCCESS) {
                // OpenCV loaded successfully
            } else {
                super.onManagerConnected(status);
            }
        }
    };

    // 静态代码块
    static {
        try {
            System.loadLibrary("omp");
            System.loadLibrary("native-lib");
            System.loadLibrary("grid_map_builder");
        } catch (UnsatisfiedLinkError e) {
            // Library loading failed
        }
    }

    // ==================== Activity生命周期 ====================
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        // 设置全屏和无标题
        this.getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);
        this.requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.activity_vslam_activity);

        // 初始化基础组件
        debugTextView = findViewById(R.id.debugTextView);
        permissionsGranted = getPermissionCamera(this);

        // 初始化UI和管理器
        initializeBasicViews();
        initializeManagers();
        initializeFilePaths();

        // 启动渲染和连接处理
        mRenderHandler = new Handler(Looper.getMainLooper());
        startPreviewRendering();
        setupDelayedConnections();

        // 注册USB广播接收器
        IntentFilter filter = new IntentFilter(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        registerReceiver(mUsbReceiver, filter);

        headingTTS = new HeadingTTS(this);
        // 初始化震动器
        vibrator = (Vibrator) getSystemService(Context.VIBRATOR_SERVICE);
        // 测试OpenMP
        try {
            testOpenMP();
        } catch (UnsatisfiedLinkError e) {
            // OpenMP test failed
        }
    }

    @Override
    public void onResume() {
        super.onResume();

        // 重新启动渲染
        if (!mIsRendering) {
            mIsRendering = true;
            startPreviewRendering();
        }

        // 重新注册USB监控
        if (cameraManager != null) {
            cameraManager.registerUSBMonitor();
            Handler handler = new Handler(Looper.getMainLooper());
            handler.postDelayed(() -> cameraManager.handleCameraConnection(), 1000);
        }

        // 初始化OpenCV
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, this, mLoaderCallback);
        } else {
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    @Override
    public void onPause() {
        mIsRendering = false;
        super.onPause();

        if (cameraManager != null) {
            cameraManager.unregisterUSBMonitor();
        }
    }

    @Override
    public void onDestroy() {
        mIsRendering = false;
        stopRosPublishing();

        // 清理资源
        cleanupResources();

        // 关闭SLAM
        if (slamInitialized) {
            try {
                shutdownSLAM(trajectoryPath);
            } catch (Exception e) {
                // SLAM shutdown error
            }
        }

        executor.shutdown();
        super.onDestroy();
    }

    // ==================== 权限相关方法 ====================
    public boolean getPermissionCamera(Activity activity) {
        List<String> permissions = new ArrayList<>();

        if (ContextCompat.checkSelfPermission(activity, Manifest.permission.CAMERA)
                != PackageManager.PERMISSION_GRANTED) {
            permissions.add(Manifest.permission.CAMERA);
        }

        if (ContextCompat.checkSelfPermission(activity, Manifest.permission.READ_EXTERNAL_STORAGE)
                != PackageManager.PERMISSION_GRANTED) {
            permissions.add(Manifest.permission.READ_EXTERNAL_STORAGE);
        }

        if (ContextCompat.checkSelfPermission(activity, Manifest.permission.WRITE_EXTERNAL_STORAGE)
                != PackageManager.PERMISSION_GRANTED) {
            permissions.add(Manifest.permission.WRITE_EXTERNAL_STORAGE);
        }

        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.TIRAMISU) {
            if (ContextCompat.checkSelfPermission(activity, Manifest.permission.READ_MEDIA_IMAGES)
                    != PackageManager.PERMISSION_GRANTED) {
                permissions.add(Manifest.permission.READ_MEDIA_IMAGES);
            }
            if (ContextCompat.checkSelfPermission(activity, Manifest.permission.READ_MEDIA_VIDEO)
                    != PackageManager.PERMISSION_GRANTED) {
                permissions.add(Manifest.permission.READ_MEDIA_VIDEO);
            }
        }

        if (!permissions.isEmpty()) {
            ActivityCompat.requestPermissions(activity,
                    permissions.toArray(new String[0]),
                    PERMISSION_REQUEST_CODE);
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        if (requestCode == PERMISSION_REQUEST_CODE) {
            boolean allGranted = true;
            for (int result : grantResults) {
                if (result != PackageManager.PERMISSION_GRANTED) {
                    allGranted = false;
                    break;
                }
            }

            if (allGranted) {
                permissionsGranted = true;
            } else {
                permissionsGranted = false;
                finish();
            }
        }
    }

    // ==================== UI和布局初始化方法 ====================
    private void initializeBasicViews() {
        // 相机视图
        mUVCCameraView = findViewById(R.id.mOpenCvCameraView);
        mUVCCameraView.getHolder().addCallback(this);

        // 轨迹控制
        showTrajectoryCheckBox = findViewById(R.id.showTrajectoryCheckBox);
        showTrajectoryCheckBox.setChecked(showTrajectory);
        showTrajectoryCheckBox.setOnCheckedChangeListener((buttonView, isChecked) -> {
            showTrajectory = isChecked;
            uiHelper.setTrajectoryVisibility(isChecked);
        });

        // 清除轨迹按钮
        clearTrajectoryButton = findViewById(R.id.clearTrajectoryButton);
        clearTrajectoryButton.setOnClickListener(v -> clearTrajectory());

        // 地图数量显示
        setupMapCountTextView();

        // 获取根视图并添加组件
        ViewGroup rootView = findViewById(android.R.id.content);
        initializeMapDisplay();

        // 创建模式状态文本视图
        modeStatusTextView = new TextView(this);
        modeStatusTextView.setTextColor(Color.WHITE);
        modeStatusTextView.setBackgroundColor(Color.argb(150, 0, 0, 0));
        modeStatusTextView.setPadding(10, 5, 10, 5);
        modeStatusTextView.setTextSize(16);
        modeStatusTextView.setText("当前模式: 未选择");

        // 添加到界面上
        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT,
                ViewGroup.LayoutParams.WRAP_CONTENT);
        params.gravity = Gravity.BOTTOM | Gravity.CENTER_HORIZONTAL;
        params.bottomMargin = 160;  // 底部边距
        rootView.addView(modeStatusTextView, params);
        // 添加模式选择按钮
        Button selectModeButton = new Button(this);
        selectModeButton.setText("选择模式");
        selectModeButton.setTextSize(12);
        selectModeButton.setOnClickListener(v -> showModeSelectionDialog());

        FrameLayout.LayoutParams buttonParams = new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT,
                ViewGroup.LayoutParams.WRAP_CONTENT);
        buttonParams.gravity = Gravity.BOTTOM | Gravity.CENTER_HORIZONTAL;
        buttonParams.bottomMargin = 220;  // 底部边距
        rootView.addView(selectModeButton, buttonParams);



        addButton(rootView, "保存地图", Gravity.BOTTOM | Gravity.END, 0, 0, 20, 200, v -> saveMap());
        addButton(rootView, "加载地图", Gravity.BOTTOM | Gravity.END, 0, 0, 20, 500, v -> {
            if (mapDisplayContainer.getVisibility() == View.VISIBLE) {
                mapDisplayContainer.setVisibility(View.GONE);
            } else {
                manualLoadMap(); // 显示地图
            }
        });


// 添加在 initializeBasicViews()
        headingAngleTextView = new TextView(this);
        headingAngleTextView.setTextColor(Color.WHITE);
        headingAngleTextView.setTextSize(14);
        headingAngleTextView.setBackgroundColor(Color.argb(150, 0, 0, 0));
        headingAngleTextView.setPadding(10, 5, 10, 5);
        headingAngleTextView.setText("路径夹角: --°");

        FrameLayout.LayoutParams angleParams = new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT,
                ViewGroup.LayoutParams.WRAP_CONTENT);
        angleParams.gravity = Gravity.TOP | Gravity.START;
        angleParams.setMargins(20, 280, 0, 0); // 放在地图数下方
        ((ViewGroup) findViewById(android.R.id.content)).addView(headingAngleTextView, angleParams);
        addButton(rootView, "初始化", Gravity.BOTTOM | Gravity.END, 0, 0, 260, 20, v -> restartApp());
    }
    private void hideMap() {
        if (mapDisplayContainer != null) {
            mapDisplayContainer.setVisibility(View.GONE);
        }
    }
    private void showModeSelectionDialog() {
        new AlertDialog.Builder(this)
                .setTitle("选择模式")
                .setItems(new String[]{"建图模式", "导航模式"}, (dialog, which) -> {
                    switch (which) {
                        case 0: // 建图模式
                            switchToConfig("PARAconfig.yaml");
                            break;
                        case 1: // 导航模式
                            switchToConfig("PARAconfig_guid.yaml");
                            break;
                    }
                })
                .setCancelable(false) // 禁止用户取消对话框
                .show();
    }

    private void switchToConfig(String filename) {
        isNavigationMode = filename.contains("guid");
        File externalDir = getExternalFilesDir(null);
        if (externalDir == null) externalDir = getFilesDir();

        File newConfigFile = new File(externalDir, "SLAM/Calibration/" + filename);
        if (!newConfigFile.exists()) {
            new AlertDialog.Builder(this)
                    .setTitle("配置文件不存在")
                    .setMessage("无法找到配置文件: " + filename)
                    .setPositiveButton("确定", null)
                    .show();
            return;
        }

        currentConfigPath = newConfigFile.getAbsolutePath();
        modeStatusTextView.setText("当前模式: " + (filename.contains("guid") ? "导航模式" : "建图模式"));

        // 初始化 SLAM
        initializeSLAMInBackground();
    }

    private void initializeMapDisplay() {
        mapDisplayContainer = findViewById(R.id.mapDisplayContainer);
        mapImageView = findViewById(R.id.mapImageView);

        mapDisplayContainer.setOnLongClickListener(v -> {
            mapDisplayContainer.setVisibility(View.GONE);
            return true;
        });
    }

    // ==================== 新增：对象文件读取相关方法 ====================
    /**
     * 读取objects.txt文件并解析对象数据
     */
    private Map<Integer, ObjectData> loadObjectsFromFile() {
        Map<Integer, ObjectData> objectMap = new HashMap<>();

        try {
            File objectsFile = new File(objectsFilePath);
            if (!objectsFile.exists()) {
                return objectMap;
            }

            Scanner scanner = new Scanner(objectsFile);
            int lineCount = 0;

            while (scanner.hasNextLine()) {
                String line = scanner.nextLine().trim();
                lineCount++;

                if (line.isEmpty()) {
                    continue;
                }

                try {
                    // 解析格式: ID,名称,X坐标,Y坐标,Z坐标
                    String[] parts = line.split(",");
                    if (parts.length >= 5) {
                        int id = Integer.parseInt(parts[0].trim());
                        String name = parts[1].trim();
                        double x = Double.parseDouble(parts[2].trim());
                        double y = Double.parseDouble(parts[3].trim());
                        double z = Double.parseDouble(parts[4].trim());

                        ObjectData object = new ObjectData(id, name, x, y, z);
                        objectMap.put(id, object);
                    }
                } catch (NumberFormatException e) {
                    // 数据解析错误
                }
            }

            scanner.close();
        } catch (Exception e) {
            // 读取文件失败
        }

        return objectMap;
    }

    /**
     * 获取指定ID的对象坐标
     */
    private ObjectData getTargetObject(int targetId) {
        Map<Integer, ObjectData> objectMap = loadObjectsFromFile();
        return objectMap.get(targetId);
    }

    /**
     * 测试读取objects.txt文件的方法
     */
    private void testLoadObjects() {
        runInBackground(() -> {
            Map<Integer, ObjectData> objectMap = loadObjectsFromFile();
        });
    }

    /**
     * 只要相机当前位置与上次规划起点不同，就重规划一次
     */
    private void checkAndReplanIfNeeded() {
        if (currentTargetObjectId == -1) return;
        double[] cur = getCurrentPosition();
        if (lastPlanningPosition == null) {
            // 如果上次规划起点为空，直接更新为当前位置
            lastPlanningPosition = cur;
            return;
        }

        // 计算当前位置与上次规划起点的欧氏距离
        double dx = cur[0] - lastPlanningPosition[0];
        double dy = cur[1] - lastPlanningPosition[1];
        double dz = cur[2] - lastPlanningPosition[2];

        double distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

        // 如果距离超过阈值，则触发重规划
        if (distance > REPLAN_THRESHOLD) {
            replanPath(currentTargetObjectId);
        }
    }

    private void replanPath(int objectId){
        // 更新起点记录
        lastPlanningPosition = getCurrentPosition();
        planPathToObject(objectId);        // 递归使用同一逻辑
    }

    /**
     * 获取当前机器人位置（仅坐标）
     * @return 位置数组 [x, y, z] 或 [0, 0, 0]
     */
    public double[] getCurrentPosition() {
        synchronized(rosDataLock) {
            if (lastCameraPose != null && lastCameraPose.length >= 3) {
                return new double[]{lastCameraPose[0], lastCameraPose[1], lastCameraPose[2]};
            }
        }
        return new double[]{0.0, 0.0, 0.0};
    }


    /**
     * 根据对象ID规划路径到指定目标
     * @param objectId 目标对象的ID（来自objects.txt文件）
     */
    public void planPathToObject(int objectId) {
        // 前置检查
        if (pathPlanningManager == null || !mapServerManager.isMapLoaded() ||
                pathPlanningManager.isPathPlanningInProgress() || !permissionsGranted) {
            return;
        }

        // 获取目标
        ObjectData target = getTargetObject(objectId);
        if (target == null) {
            return;
        }

        // ==================== 重置所有状态 ====================
        hasReachedTarget = false;  // 重置到达目标状态
        hasPlayedLocationSuccessAudio = false;  // 重置定位成功语音状态
        previousMapCount = -1;  // 重置地图数状态

        // 更新路径规划状态
        isPathPlanningActive = true;
        currentTargetObjectId = objectId;

        // 获取当前相机位置作为起点
        lastPlanningPosition = getCurrentPosition();

        // 规划路径
        double[] curPos = lastPlanningPosition;
        pathPlanningManager.planPath(
                curPos[0], curPos[1],
                target.x, target.y,
                new PathPlanningManager.PathPlanningCallback() {
                    @Override
                    public void onPathPlanningStarted() {
                        // 开始规划路径的逻辑
                    }

                    @Override
                    public void onPathPlanningCompleted(List<PathPlanningManager.PathPoint> path) {
                        runOnUI(() -> {
                            isPathPlanningActive = false;
                        });
                    }

                    @Override
                    public void onPathPlanningFailed(String error) {
                        runOnUI(() -> {
                            isPathPlanningActive = false;
                        });
                    }

                    @Override
                    public void onPathOptimizationCompleted(List<PathPlanningManager.PathPoint> original,
                                                            List<PathPlanningManager.PathPoint> optimized) {
                        // 路径优化完成的逻辑
                    }
                }
        );
    }

    public boolean isNavigationMode() {
        return isNavigationMode;
    }

    private void setupMapCountTextView() {
        mapCountTextView = new TextView(this);
        mapCountTextView.setTextColor(Color.WHITE);
        mapCountTextView.setTextSize(16);
        mapCountTextView.setBackgroundColor(Color.argb(128, 0, 0, 0));
        mapCountTextView.setPadding(10, 5, 10, 5);
        mapCountTextView.setText("地图数: 0");

        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT,
                ViewGroup.LayoutParams.WRAP_CONTENT);
        params.gravity = Gravity.TOP | Gravity.END;
        params.setMargins(0, 180, 20, 0);

        ViewGroup rootView = findViewById(android.R.id.content);
        rootView.addView(mapCountTextView, params);
    }

    private void addButton(ViewGroup parent, String text, int gravity,
                           int leftMargin, int topMargin, int rightMargin, int bottomMargin,
                           View.OnClickListener listener) {
        Button button = new Button(this);
        button.setText(text);
        button.setTextSize(12);
        button.setOnClickListener(listener);

        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(
                ViewGroup.LayoutParams.WRAP_CONTENT,
                ViewGroup.LayoutParams.WRAP_CONTENT);
        params.gravity = gravity;
        params.setMargins(leftMargin, topMargin, rightMargin, bottomMargin);
        parent.addView(button, params);
    }

    private void initializeManagers() {
        // 相机管理器
        cameraManager = new USBCameraManager(this, this);

        // ROS和地图管理器
        rosManager = new RosManager(this);
        gridMapManager = new GridMapManager(this, rosManager);
        rosPublishHandler = new Handler();

        // UI辅助器
        uiHelper = new VslamUIHelper(this);
        uiHelper.setupAllUI();

        // 地图服务器和目标发布器
        mapServerManager = new MapServerManager(this, rosManager);
        mapServerManager.getScaleController().setBaseScale(5.0f);//放大地图显示
        mapServerManager.setMapImageView(mapImageView);


        goalPublisher = new GoalPublisher(this, rosManager);

        // 初始化路径规划管理器
        pathPlanningManager = new PathPlanningManager(this, mapServerManager);
    }

    private void setupDelayedConnections() {
        Handler handler = new Handler();
        handler.postDelayed(() -> {
            if (isRosConnected() && !isRosPublishing()) {
                startRosPublishing();
                rosManager.testPublishLocalPointCloud();
            }
        }, 2000);
    }

    // ==================== Camera相关回调 ====================
    @Override
    public void onCameraConnected(UVCCamera camera) {
        // 不立即初始化SLAM，等待画面稳定
        if (!slamInitialized) {
            // 延迟1秒后自动初始化SLAM，让画面先稳定
            Handler handler = new Handler(Looper.getMainLooper());
            handler.postDelayed(() -> {
                if (!slamInitialized && cameraManager != null) {
                    //initializeSLAMInBackground();
                }
            }, 1);
        }
    }

    @Override
    public void onCameraDisconnected() {
        // USB相机已断开连接
    }

    @Override
    public void onCameraAttach(UsbDevice device) {
        // USB设备已连接
    }

    @Override
    public void onCameraDetach(UsbDevice device) {
        // USB设备已移除
    }

    @Override
    public void onConnectFail(String reason) {
        // 相机连接失败
    }

    @Override
    public void onShowCameraDialog() {
        CameraDialog.showDialog(this);
    }

    @Override
    public void onFrameReceived(ByteBuffer frame) {
        try {
            // 处理YUV数据
            byte[] yuv = new byte[frame.remaining()];
            frame.get(yuv);
            frame.rewind();

            // 创建Mat并转换为RGB
            Mat yuvMat = new Mat(CAMERA_HEIGHT + CAMERA_HEIGHT/2, CAMERA_WIDTH, CvType.CV_8UC1);
            yuvMat.put(0, 0, yuv);

            Mat rgb = new Mat();
            Imgproc.cvtColor(yuvMat, rgb, Imgproc.COLOR_YUV2RGB_NV21);
            yuvMat.release();

            // 旋转图像
            Core.rotate(rgb, rgb, Core.ROTATE_180);
            count++;

            // 保存当前帧
            synchronized (this) {
                mCurrentFrame = rgb.clone();
            }

            // 分离左右图像
            processStereoPair(rgb);

            rgb.release();
        } catch (Exception e) {
            // Frame processing error
        }
    }

    private void processStereoPair(Mat rgb) {
        org.opencv.core.Rect leftROI = new org.opencv.core.Rect(0, 0, rgb.width()/2, rgb.height());
        org.opencv.core.Rect rightROI = new org.opencv.core.Rect(rgb.width()/2, 0, rgb.width()/2, rgb.height());

        Mat leftFrame = new Mat(rgb, leftROI);
        Mat rightFrame = new Mat(rgb, rightROI);

        // 发布到ROS
        if (rosManager != null && rosManager.isConnected()) {
            Mat leftCopy = leftFrame.clone();
            Mat rightCopy = rightFrame.clone();
            rosManager.publishStereoImages(leftCopy, rightCopy);
            leftCopy.release();
            rightCopy.release();
        }

        // 处理SLAM或预览
        if (slamInitialized) {
            processFrameWithPreview(rgb);
        } else {
            updatePreviewBitmap(rgb);
        }

        leftFrame.release();
        rightFrame.release();
    }

    @Override
    public void runInBackground(Runnable task) {
        executor.execute(task);
    }

    @Override
    public void runOnUIThread(Runnable task) {
        mainHandler.post(task);
    }

    // ==================== SLAM相关处理 ====================
    private void processFrameWithPreview(final Mat rgb) {
        if (rgb == null || rgb.empty() || isRestarting) return;
        synchronized (slamLock) {
            if (!slamInitialized) {
                return;
            }
            try {
                // 分离左右图像
                org.opencv.core.Rect leftROI = new org.opencv.core.Rect(0, 0, rgb.width() / 2, rgb.height());
                org.opencv.core.Rect rightROI = new org.opencv.core.Rect(rgb.width() / 2, 0, rgb.width() / 2, rgb.height());

                Mat leftFrame = new Mat(rgb, leftROI);
                Mat rightFrame = new Mat(rgb, rightROI);

                // 处理SLAM帧
                double timestamp = System.currentTimeMillis() / 1000.0;
                final float[] poseMatrix = processStereoFrame(leftFrame.getNativeObjAddr(),
                        rightFrame.getNativeObjAddr(), timestamp);

                // 处理深度图
                handleDepthImageGeneration(leftFrame);

                // 处理关键帧
                handleKeyFrameProcessing();

                // 处理地图点
                float[] mapPoints = getTrackedMapPoints();

                // 处理姿态和轨迹
                handlePoseAndTrajectory(poseMatrix, mapPoints);

                // 更新预览
                updatePreviewBitmap(rgb);

                // 更新地图数量
                updateMapCount();

                leftFrame.release();
                rightFrame.release();
            } catch (Exception e) {
                // Frame processing error
            }
        }
    }

    private void handleDepthImageGeneration(Mat leftFrame) {
        if (slamInitialized && rosManager != null && rosManager.isConnected()) {
            try {
                if (rosManager.hasDepthImageSubscribers()) {
                    long depthMatAddr = generateDepthMapFromSLAM(leftFrame.getNativeObjAddr());
                    if (depthMatAddr != 0) {
                        rosManager.publishDepthImage(depthMatAddr);
                        releaseDepthMap(depthMatAddr);
                    }
                }
            } catch (Exception e) {
                // Depth image processing error
            }
        }
    }

    private void handleKeyFrameProcessing() {
        boolean isNewKF = isNewKeyFrame();
        if (isNewKF && !lastKeyFrameState) {
            float[] keyFrameData = getCurrentKeyFrameData();
            if (keyFrameData != null && keyFrameData.length > 7) {
                rosManager.publishKeyFrameData(keyFrameData);
                if (gridMapManager != null) {
                    gridMapManager.processKeyframeData(keyFrameData);
                }
            }
            lastKeyFrameState = true;
        } else {
            lastKeyFrameState = false;
        }
    }

    private void handlePoseAndTrajectory(float[] poseMatrix, float[] mapPoints) {
        if (poseMatrix != null && poseMatrix.length >= 7) {
            double[] position = new double[3];
            position[0] = poseMatrix[0];
            position[1] = poseMatrix[1];
            position[2] = poseMatrix[2];
            if (pathPlanningManager != null) {
                pathPlanningManager.updateRobotPose(poseMatrix);
            }
            // 更新轨迹
            if (count % TRAJECTORY_SAMPLING == 0) {
                trajectoryPoints.add(position);
                if (trajectoryPoints.size() > MAX_TRAJECTORY_POINTS) {
                    trajectoryPoints.remove(0);
                }
                if (showTrajectory) {
                    runOnUI(() -> uiHelper.invalidateTrajectoryView());
                }
            }

            // 更新ROS数据
            if (useROS && poseMatrix.length >= 7) {
                float[] cameraPose = poseMatrix.clone();
                synchronized(rosDataLock) {
                    lastCameraPose = cameraPose;
                    if (mapPoints != null && mapPoints.length > 0) {
                        lastMapPoints = mapPoints.clone();
                    }
                }
            }
        }

        if (mapServerManager.isImageViewSet()) {
            Bitmap mapBitmap = mapServerManager.getMapDisplayBitmap();
            if (mapBitmap != null) {
                Bitmap mutable = mapBitmap.copy(Bitmap.Config.ARGB_8888, true);
                Canvas canvas = new Canvas(mutable);
                pathPlanningManager.redrawPathAndArrow(canvas, lastPlanningPosition, lastCameraPose);
                mapServerManager.updateMapDisplayBitmap(mutable);
            }
        }

        // 检查是否需要重新规划
        if (currentTargetObjectId != -1) {
            checkAndReplanIfNeeded();
        }

        // ==================== 修改后的语音播报逻辑 ====================
        if (pathPlanningManager != null && lastCameraPose != null) {
            List<PathPlanningManager.PathPoint> path = pathPlanningManager.getCurrentPath();
            if (path == null || path.isEmpty()) return;

            final float angle = pathPlanningManager.calculatePathHeadingAngle(path, lastCameraPose);

            // 使用第一段路径长度（与角度计算保持一致）
            double segmentLen = 0.0;
            if (path.size() >= 2) {
                PathPlanningManager.PathPoint realStart = path.get(path.size() - 1);  // 真正的起点
                PathPlanningManager.PathPoint realNext = path.get(path.size() - 2);   // 起点的下一个点
                double dx = realNext.worldX - realStart.worldX;
                double dy = realNext.worldY - realStart.worldY;
                segmentLen = Math.sqrt(dx * dx + dy * dy);  // **第一段路径的长度**
            }

            final double finalLen = segmentLen;

            runOnUI(() -> {
                // 获取当前地图数
                int currentMapCount = 0;
                try {
                    currentMapCount = getMapCount();
                } catch (Exception e) {
                    currentMapCount = 0;
                }

                // ==================== 声明为final变量供lambda使用 ====================
                final int finalCurrentMapCount = currentMapCount;

                // 计算到目标的距离（使用路径长度而不是直线距离）
                double distanceToTarget = Double.MAX_VALUE;
                if (currentTargetObjectId != -1 && pathPlanningManager != null) {
                    // 使用优化后的路径长度作为距离
                    distanceToTarget = pathPlanningManager.getCurrentPathLength();
                }

                final double finalDistanceToTarget = distanceToTarget;

                // ==================== 检测定位状态变化 ====================
                boolean locationJustSucceeded = false;
                if (previousMapCount != 1 && currentMapCount == 1) {
                    // 地图数刚从非1变为1，表示定位刚成功
                    locationJustSucceeded = true;
                    hasPlayedLocationSuccessAudio = false;  // 重置定位成功语音播报状态
                }
                previousMapCount = currentMapCount;  // 更新状态

                long now = System.currentTimeMillis();
                if (now - lastTriggerMillis < 1000) return;   // 1 秒内忽略
                lastTriggerMillis = now;

                // ==================== 添加直行条件的到达目标判断 ====================
                final float STRAIGHT_ARRIVAL_ANGLE = 15.0f; // 直行判定角度阈值
                boolean isCloseEnough = finalDistanceToTarget <= TARGET_REACH_DISTANCE;
                boolean isStraightDirection = Math.abs(angle) < STRAIGHT_ARRIVAL_ANGLE;
                boolean isReallyArrived = isCloseEnough && isStraightDirection;

                // ==================== 语音播报逻辑 ====================
                if (currentMapCount == 1) {
                    // mapCount = 1 时（定位成功状态）

                    if (isReallyArrived && !hasReachedTarget) {
                        // ✅ 条件: mapCount=1 AND 距离<0.5米 AND 角度<15度 AND 未播报过 → 播报"已到达目标"
                        headingAngleTextView.setText(String.format(Locale.CHINA,
                                "已到达目标 (距离: %.2f米, 角度: %.1f°)", finalDistanceToTarget, Math.abs(angle)));
                        headingTTS.speakHeading("已到达目标");
                        hasReachedTarget = true;

                        // 到达目标时的震动反馈
                        if (vibrator != null && vibrator.hasVibrator()) {
                            vibrator.vibrate(500); // 震动500毫秒
                        }

                    } else if (isReallyArrived && hasReachedTarget) {
                        // ✅ 条件: mapCount=1 AND 距离<0.5米 AND 角度<15度 AND 已播报过 → 保持静默
                        headingAngleTextView.setText(String.format(Locale.CHINA,
                                "已到达目标 (距离: %.2f米, 角度: %.1f°)", finalDistanceToTarget, Math.abs(angle)));
                        // 不播报任何语音

                    } else {
                        // ✅ 其他情况 → 播报导航信息
                        // 如果之前到达过目标但现在条件不满足了，重置状态
                        if (hasReachedTarget) {
                            hasReachedTarget = false;
                        }

                        // 特殊提示：距离够了但角度不对
                        if (isCloseEnough && !isStraightDirection) {
                            headingAngleTextView.setText(String.format(Locale.CHINA,
                                    "接近目标，请调整方向 (距离: %.2f米, 偏转: %.1f°)",
                                    finalDistanceToTarget, Math.abs(angle)));
                        }

                        playNavigationAudio(angle, finalLen, finalDistanceToTarget);
                    }

                } else if (currentMapCount > 1) {
                    // 条件: mapCount>1 → 播报定位提示（无论距离多远）
                    headingAngleTextView.setText("定位中... 请轻轻摆动相机");

                    // 重置定位成功语音状态（因为又失去定位了）
                    hasPlayedLocationSuccessAudio = false;
                    // 重置到达目标状态
                    if (hasReachedTarget) {
                        hasReachedTarget = false;
                    }

                    // 语音播报定位提示（3秒间隔）
                    boolean shouldSpeak = firstSpeak ||
                            (now - lastTriggerMillis >= 3000);

                    if (shouldSpeak) {
                        headingTTS.speakHeading("定位中... 请轻轻摆动相机");
                        firstSpeak = false;
                    }

                } else {
                    // 条件: mapCount=0或其他 → 显示状态但不播报
                    headingAngleTextView.setText("地图数: " + currentMapCount + " - 系统准备中");
                    hasPlayedLocationSuccessAudio = false;  // 重置状态
                    // 重置到达目标状态
                    if (hasReachedTarget) {
                        hasReachedTarget = false;
                    }
                }
            });
        }
    }

    // ==================== 导航语音播报方法 ====================

    // ==================== 添加语音控制相关成员变量 ====================
// 在类的成员变量区域，只添加缺少的变量：
    private boolean isSpeaking = false;
    private String lastSpokenText = "";
    private long lastSpeakTime =0;
    private static final long SPEAK_INTERVAL = 3000; // 3秒间隔
    private static final float ANGLE_CHANGE_THRESHOLD = 15.0f; // 角度变化阈值5度

// 如果lastSpokenAngle已存在就不要重复添加，如果不存在才添加：
// private float lastSpokenAngle = Float.NaN; // 记录上次播报的角度

    private void playNavigationAudio(float angle, double finalLen, double distanceToTarget) {
        String directionText;
        String voiceText;

        // 生成显示和语音文本
        if (Math.abs(angle) < 15.0f) {
            directionText = String.format(Locale.CHINA, "向前直行 %.1f 米 (距目标路径: %.2f米)", finalLen, distanceToTarget);
            voiceText = String.format(Locale.CHINA, "向前直行%.1f米", finalLen);
        } else if (angle >= 0) {
            directionText = String.format(Locale.CHINA, "向右偏转 %.1f° (距目标路径: %.2f米)", Math.abs(angle), distanceToTarget);
            voiceText = String.format(Locale.CHINA, "向右偏转%.1f度", Math.abs(angle));
        } else {
            directionText = String.format(Locale.CHINA, "向左偏转 %.1f° (距目标路径: %.2f米)", Math.abs(angle), distanceToTarget);
            voiceText = String.format(Locale.CHINA, "向左偏转%.1f度", Math.abs(angle));
        }

        // 更新界面显示（这个可以频繁更新）
        headingAngleTextView.setText(directionText);

        // 语音播报控制 - 修正逻辑，角度阈值优先
        long currentTime = System.currentTimeMillis();
        boolean shouldSpeak = false;
        String debugReason = ""; // 用于调试

        if (!isSpeaking) {
            // 情况1：首次播报
            if (Float.isNaN(lastSpokenAngle)) {
                shouldSpeak = true;
                debugReason = "首次播报";
            }
            // 情况2：角度变化超过阈值（主要条件）
            else if (Math.abs(angle - lastSpokenAngle) >= ANGLE_CHANGE_THRESHOLD) {
                shouldSpeak = true;
                debugReason = String.format("角度变化%.1f°超过阈值%.1f°",
                        Math.abs(angle - lastSpokenAngle), ANGLE_CHANGE_THRESHOLD);
            }
            // 情况3：方向发生根本性变化（从左转变右转等）
            else {
                String currentDirection = getDirection(angle);
                String lastDirection = getDirection(lastSpokenAngle);
                if (!currentDirection.equals(lastDirection)) {
                    shouldSpeak = true;
                    debugReason = String.format("方向变化: %s -> %s", lastDirection, currentDirection);
                }
            }

            // 情况4：超长时间没播报了（保险机制，避免长时间静音）
            // 只有在角度变化不大但时间很长时才触发
            if (!shouldSpeak && (currentTime - lastSpeakTime > SPEAK_INTERVAL * 2)) {
                shouldSpeak = true;
                debugReason = "超长时间未播报";
            }
        }

        if (shouldSpeak) {
            isSpeaking = true;
            lastSpokenText = voiceText;
            lastSpokenAngle = angle; // 记录播报时的角度
            lastSpeakTime = currentTime;

            // 播放语音
            headingTTS.speakHeading(voiceText);

            // 估算播放时长并重置状态
            int estimatedDuration = Math.max(2000, voiceText.length() * 100);
            new Handler().postDelayed(() -> {
                isSpeaking = false;
            }, estimatedDuration);

            // 转弯时震动
            if (Math.abs(angle) >= 15.0f && vibrator != null && vibrator.hasVibrator()) {
                vibrator.vibrate(200);
            }

            // 临时调试信息（你可以在测试完后删除这行）
            // System.out.println("语音播报: " + debugReason + " | 角度=" + angle + "° | 内容=" + voiceText);
        }
    }

    // 获取角度对应的方向类型
    private String getDirection(float angle) {
        if (Float.isNaN(angle)) return "unknown";
        if (Math.abs(angle) < 15.0f) return "straight";
        return angle >= 0 ? "right" : "left";
    }

    private void updateMapCount() {
        try {
            int mapCount = getMapCount();
            runOnUI(() -> mapCountTextView.setText("地图数: " + mapCount));
            if (rosManager != null && rosManager.isConnected()) {
                rosManager.publishMapCount(mapCount);
            }
        } catch (Exception e) {
            // Map count error
        }
    }

    private void initializeSLAMInBackground() {
        runOnUI(() -> {
            final ProgressDialog dialog = new ProgressDialog(this);
            dialog.setTitle("初始化SLAM");
            dialog.setMessage("正在准备SLAM系统...\n初始化可能需要较长时间，请耐心等待");
            dialog.setCancelable(false);
            dialog.show();

            runInBackground(() -> {
                synchronized (slamLock) {
                    boolean success = false;
                    String errorMessage = "";

                    try {
                        // 验证文件存在
                        validateSLAMFiles();

                        // 处理词汇表
                        handleVocabularyFile(dialog);

                        // 初始化 SLAM
                        runOnUI(() -> dialog.setMessage("正在初始化SLAM系统...\n这可能需要几分钟，请耐心等待"));
                        boolean initResult = initializeSLAM(vocPath, currentConfigPath);
                        if (!initResult) {
                            throw new Exception("初始化SLAM系统失败");
                        }

                        Thread.sleep(3000); // 模拟初始化耗时
                        success = true;
                        slamInitialized = true;

                    } catch (Exception e) {
                        errorMessage = e.getMessage();
                    } finally {
                        handleSLAMInitializationResult(dialog, success, errorMessage);
                    }
                }
            });
        });
    }


    private void validateSLAMFiles() throws Exception {
        File vocFile = new File(vocPath);
        File configFile = new File(configPath);

        if (!vocFile.exists() || vocFile.length() == 0) {
            throw new Exception("VOC文件不存在或为空");
        }

        if (!configFile.exists() || configFile.length() == 0) {
            throw new Exception("CONFIG文件不存在或为空");
        }
    }

    private void handleVocabularyFile(ProgressDialog dialog) {
        String txtPath = vocPath;
        String binPath = txtPath.replace("ORBvoc.txt", "ORBvoc.bin");
        File binFile = new File(binPath);

        if (binFile.exists() && binFile.length() > 0) {
            vocPath = binPath;
        } else {
            runOnUI(() -> dialog.setMessage("正在转换词汇表文件...\n这可能需要几分钟"));
            boolean conversionResult = convertVocabulary(txtPath, binPath);
            if (conversionResult) {
                vocPath = binPath;
            }
        }
    }

    private void handleSLAMInitializationResult(ProgressDialog dialog, boolean success, String errorMessage) {
        runOnUI(() -> {
            dialog.dismiss();
            if (success) {
                // SLAM初始化成功
            } else {
                new AlertDialog.Builder(this)
                        .setTitle("SLAM初始化失败")
                        .setMessage("错误信息: " + errorMessage)
                        .setPositiveButton("确定", null)
                        .show();
                myTextView.setText("SLAM状态: 初始化失败");
            }
        });
    }

    // ==================== 渲染与显示相关 ====================
    private void startPreviewRendering() {
        mIsRendering = true;
        final Runnable renderRunnable = new Runnable() {
            private long lastFrameTime = 0;

            @Override
            public void run() {
                if (!mIsRendering) return;

                long currentTime = System.currentTimeMillis();
                if (currentTime - lastFrameTime < 16) {
                    mRenderHandler.postDelayed(this, 16 - (currentTime - lastFrameTime));
                    return;
                }
                lastFrameTime = currentTime;

                updateFPSStats();

                synchronized (mBitmapLock) {
                    if (mLatestBitmap != null && !mLatestBitmap.isRecycled() &&
                            mUVCCameraView != null && mUVCCameraView.getHolder().getSurface().isValid()) {
                        try {
                            Canvas canvas = mUVCCameraView.getHolder().lockCanvas();
                            if (canvas != null) {
                                canvas.drawColor(Color.BLACK, PorterDuff.Mode.CLEAR);
                                drawCameraImage(canvas);

                                if (showTrajectory) {
                                    drawTrajectory(canvas);
                                }

                                mUVCCameraView.getHolder().unlockCanvasAndPost(canvas);
                            }
                        } catch (Exception e) {
                            // Rendering error
                        }
                    }
                }

                mRenderHandler.postDelayed(this, 16);
            }
        };
        mRenderHandler.post(renderRunnable);
    }

    private void drawCameraImage(Canvas canvas) {
        if (mLatestBitmap == null) return;

        float scale = Math.min(
                (float) canvas.getWidth() / mLatestBitmap.getWidth(),
                (float) canvas.getHeight() / mLatestBitmap.getHeight());

        float left = (canvas.getWidth() - mLatestBitmap.getWidth() * scale) / 2;
        float top = (canvas.getHeight() - mLatestBitmap.getHeight() * scale) / 2;

        android.graphics.Matrix matrix = new android.graphics.Matrix();
        matrix.postScale(scale, scale);
        matrix.postTranslate(left, top);

        Paint paint = new Paint();
        paint.setFilterBitmap(true);
        paint.setAntiAlias(true);
        canvas.drawBitmap(mLatestBitmap, matrix, paint);
    }

    private void drawTrajectory(Canvas canvas) {
        if (trajectoryPoints.isEmpty()) return;

        Paint trajectoryPaint = new Paint();
        trajectoryPaint.setColor(Color.GREEN);
        trajectoryPaint.setStrokeWidth(3);
        trajectoryPaint.setAntiAlias(true);

        for (int i = 1; i < trajectoryPoints.size(); i++) {
            double[] prev = trajectoryPoints.get(i - 1);
            double[] curr = trajectoryPoints.get(i);

            float x1 = canvas.getWidth() / 2 + (float) prev[0] * 50;
            float y1 = canvas.getHeight() / 2 + (float) prev[2] * 50;
            float x2 = canvas.getWidth() / 2 + (float) curr[0] * 50;
            float y2 = canvas.getHeight() / 2 + (float) curr[2] * 50;

            canvas.drawLine(x1, y1, x2, y2, trajectoryPaint);
        }
    }

    private void updatePreviewBitmap(Mat mat) {
        if (mat == null || mat.empty()) return;

        try {
            Mat display = mat.clone();
            Bitmap newBitmap = Bitmap.createBitmap(display.cols(), display.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(display, newBitmap);

            synchronized (mBitmapLock) {
                if (mLatestBitmap != null && !mLatestBitmap.isRecycled()) {
                    mLatestBitmap.recycle();
                    mLatestBitmap = null;
                }
                mLatestBitmap = newBitmap;
            }
            display.release();
        } catch (Exception e) {
            // Bitmap update error
        }
    }

    private void updateFPSStats() {
        long currentTime = System.currentTimeMillis();
        if (lastFrameTime != 0) {
            frameCount++;
            if (frameCount % 30 == 0) {
                float timeDiff = (currentTime - lastFrameTime) / 1000.0f;
                averageFPS = 30.0f / timeDiff;
                lastFrameTime = currentTime;
            }
        } else {
            lastFrameTime = currentTime;
        }
    }

    // ==================== 轨迹相关方法 ====================
    private void clearTrajectory() {
        trajectoryPoints.clear();
        uiHelper.invalidateTrajectoryView();
    }

    // ==================== ROS与地图相关方法 ====================
    public void startRosPublishing() {
        if (isRosPublishing) return;
        isRosPublishing = true;
        rosPublishHandler.post(this::publishRosData);
    }

    public void stopRosPublishing() {
        isRosPublishing = false;
        rosPublishHandler.removeCallbacksAndMessages(null);
    }

    private void publishRosData() {
        //int mapCount1 = 10054;
        //rosManager.publishMapCount(mapCount1);

        if (!isRosPublishing || rosManager == null || !rosManager.isConnected()) {
            if (isRosPublishing) {
                rosPublishHandler.postDelayed(this::publishRosData, ROS_PUBLISH_INTERVAL);
            }
            return;
        }

        try {
            boolean isNewKF = isNewKeyFrame();

            float[] cameraPose = null;
            synchronized(rosDataLock) {
                if (lastCameraPose != null) {
                    cameraPose = lastCameraPose.clone();
                }
            }

            if (cameraPose != null) {
                rosManager.publishCurrentCameraPose(cameraPose);
            }

            if (checkLoopClosure()) {
                float[] allKeyframeData = getAllKeyframeData();
                if (allKeyframeData != null && allKeyframeData.length > 0) {
                    rosManager.publishAllKeyframeData(allKeyframeData);
                    if (gridMapManager != null) {
                        gridMapManager.processAllKeyframesData(allKeyframeData);
                    }
                }
            }
        } catch (Exception e) {
            // ROS publish error
        } finally {
            if (isRosPublishing) {
                rosPublishHandler.postDelayed(this::publishRosData, ROS_PUBLISH_INTERVAL);
            }
        }
    }

    private void loadMapIfAvailable() {
        if (mapLoadAttempted) return;
        mapLoadAttempted = true;

        runInBackground(() -> {
            try {
                File mapFile = new File(mapYamlPath);
                if (!mapFile.exists()) return;

                if (mapServerManager != null) {
                    mapServerManager.loadMapFromYaml(mapYamlPath);
                    Thread.sleep(2000);
                }
            } catch (Exception e) {
                updateDebugTextView("❌ 加载地图时出错: " + e.getMessage());
            }
        });
    }

    public void manualLoadMap() {
        if (mapServerManager != null) {
            mapLoadAttempted = false;
            loadMapIfAvailable();
            runOnUI(() -> {
                mapDisplayContainer.setVisibility(View.VISIBLE);
            });
        }
    }

    public void saveMap() {
        if (gridMapManager != null) {
            gridMapManager.saveMap();
        }
    }
    private void restartApp() {
        // 彻底关闭进程再重启（最通用、最干净、不会内存泄露）
        Intent intent = getBaseContext().getPackageManager()
                .getLaunchIntentForPackage(getBaseContext().getPackageName());
        intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP | Intent.FLAG_ACTIVITY_NEW_TASK);
        startActivity(intent);
        finishAffinity(); // 彻底关闭所有Activity
        System.exit(0);   // 彻底退出当前进程
    }

    // ==================== 文件系统相关方法 ====================
    private void initializeFilePaths() {
        File externalDir = getExternalFilesDir(null);
        if (externalDir == null) {
            externalDir = getFilesDir();
        }

        vocPath = new File(externalDir, "SLAM/VOC/ORBvoc.txt").getAbsolutePath();
        configPath = new File(externalDir, "SLAM/Calibration/PARAconfig.yaml").getAbsolutePath();
        trajectoryPath = new File(externalDir, "SLAM/Trajectory/KeyFrameTrajectory.txt").getAbsolutePath();
        mapYamlPath = new File(externalDir, "SLAM/Maps/new_map.yaml").getAbsolutePath();
        //currentConfigPath = new File(externalDir, "SLAM/Calibration/PARAconfig.yaml").getAbsolutePath(); // 默认
        // 新增：objects.txt文件路径
        objectsFilePath = new File(externalDir, "SLAM/Maps/objects.txt").getAbsolutePath();

        createRequiredDirectories();
    }

    private void createRequiredDirectories() {
        File vocDir = new File(vocPath).getParentFile();
        File configDir = new File(configPath).getParentFile();
        File trajectoryDir = new File(trajectoryPath).getParentFile();
        File mapDir = new File(mapYamlPath).getParentFile();

        if (vocDir != null) vocDir.mkdirs();
        if (configDir != null) configDir.mkdirs();
        if (trajectoryDir != null) trajectoryDir.mkdirs();
        if (mapDir != null) mapDir.mkdirs();
    }

    // ==================== Surface相关回调 ====================
    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        mPreviewSurface = holder.getSurface();
        if (cameraManager != null) {
            cameraManager.setPreviewSurface(mPreviewSurface);
        }
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        // Surface changed handling
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        mPreviewSurface = null;
    }

    // ==================== 对话框相关回调 ====================
    @Override
    public USBMonitor getUSBMonitor() {
        return cameraManager != null ? cameraManager.getUSBMonitor() : null;
    }

    @Override
    public void onDialogResult(boolean canceled) {
        // Dialog result handling
    }

    // ==================== 资源清理方法 ====================
    private void cleanupResources() {
        // 先停止所有正在进行的操作
        if (pathPlanningManager != null && pathPlanningManager.isPathPlanningInProgress()) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // 清理路径规划管理器
        if (pathPlanningManager != null) {
            pathPlanningManager.shutdown();
            pathPlanningManager = null;
        }

        // 清理地图服务器
        if (mapServerManager != null) {
            mapServerManager.shutdown();
            mapServerManager = null;
        }

        // 清理位图
        synchronized (mBitmapLock) {
            if (mLatestBitmap != null && !mLatestBitmap.isRecycled()) {
                mLatestBitmap.recycle();
                mLatestBitmap = null;
            }
        }

        // 注销广播接收器
        try {
            unregisterReceiver(mUsbReceiver);
        } catch (Exception e) {
            // Receiver already unregistered
        }

        // 清理相机管理器
        if (cameraManager != null) {
            cameraManager.destroy();
            cameraManager = null;
        }

        // 清理各种管理器
        if (gridMapManager != null) {
            gridMapManager.shutdown();
            gridMapManager = null;
        }

        if (goalPublisher != null) {
            goalPublisher.shutdown();
            goalPublisher = null;
        }

        if (rosManager != null) {
            rosManager.shutdown();
            rosManager = null;
        }
    }

    // ==================== 调试和状态更新方法 ====================
    public void updateDebugTextView(final String message) {
        runOnUiThread(() -> {
            TextView debugTextView = findViewById(R.id.debugTextView);
            debugTextView.append(message + "\n");
        });
    }

    // ==================== 工具与辅助方法 ====================
    public void runOnUI(Runnable task) {
        mainHandler.post(task);
    }

    // ==================== Getter方法 ====================
    public List<double[]> getTrajectoryPoints() {
        return trajectoryPoints;
    }

    public boolean isRosPublishing() {
        return isRosPublishing;
    }

    public boolean isRosConnected() {
        return rosManager != null && rosManager.isConnected();
    }

    public UVCCamera getUVCCamera() {
        return cameraManager != null ? cameraManager.getUVCCamera() : null;
    }

    public USBMonitor getUSBMonitorInstance() {
        return cameraManager != null ? cameraManager.getUSBMonitor() : null;
    }

    public long getFrameCount() {
        return count;
    }

    public boolean isSlamInitialized() {
        return slamInitialized;
    }

    public GridMapBuilder.GridMapInfo getMapInfo() {
        return gridMapManager != null ? gridMapManager.getMapInfo() : null;
    }

    public boolean isGridMapInitialized() {
        return gridMapManager != null;
    }
}