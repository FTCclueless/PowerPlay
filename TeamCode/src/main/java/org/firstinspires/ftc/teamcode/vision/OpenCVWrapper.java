package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class OpenCVWrapper {
    private String TAG = "OpenCVWrapper";
    OpenCvCamera phoneCam;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    MyOpenCvPipeline openCvPipeline;
    boolean useWebCamera;

    public OpenCVWrapper(Telemetry tele, HardwareMap hwMap, MyOpenCvPipeline pipeline, boolean webCam) {
        telemetry = tele;
        hardwareMap = hwMap;
        openCvPipeline = pipeline;
        useWebCamera = webCam;
    }

    public void init() {
        RobotLogger.dd(TAG, "init OpenCVWrapper");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //cone_tracker.setTelemetry(telemetry);
        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        if (!useWebCamera)
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        else
            phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Connect to the camera
        phoneCam.openCameraDevice();
        // Use the OpenCVObjectDetector pipeline
        // processFrame() will be called to process the frame
        phoneCam.setPipeline(openCvPipeline);
        openCvPipeline.setTelemetry(telemetry);
    }

    public void start() {
        RobotLogger.dd(TAG, "start OpenCVWrapper");

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    public void stop() {
        RobotLogger.dd(TAG, "stop OpenCVWrapper");
        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();
    }
}
