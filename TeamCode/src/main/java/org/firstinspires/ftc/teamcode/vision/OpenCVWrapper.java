package org.firstinspires.ftc.teamcode.vision;

import android.hardware.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.vision.ConeTracker;
import org.firstinspires.ftc.teamcode.vision.OpenCVObjectDetector;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class OpenCVWrapper {
    private String TAG = "OpenCVWrapper";
    OpenCvCamera phoneCam;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    MyOpenCvPipeline openCvPipeline;
    boolean useWebCamera;
    ArrayList<AprilTagDetection> detections = new ArrayList<>();

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
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
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

    public ArrayList<AprilTagDetection> getLatestDetections() {
        return openCvPipeline.getLatestDetections();
    }
    public int getAprilTagID() {
        ArrayList<AprilTagDetection> currentDetections = getLatestDetections();
        int ret = -1;
        if(currentDetections.size() != 0) {
            boolean tagFound = false;
            for (AprilTagDetection tag : currentDetections) {
                RobotLogger.dd("", "detected num of tagIDs " + currentDetections.size()
                    + " tagID: " + tag.id);
                ret = tag.id;
                break;
            }
        }
        return ret;
    }
    public void zoomCamera(int zoom) {
        //phoneCam.setZoom(zoom);
    }

    public OpenCvCamera getCamera() {
        return phoneCam;
    }
}
