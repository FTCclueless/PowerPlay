package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;

public class OpenCVWrapper {
    private String TAG = "OpenCVWrapper";
    OpenCvCamera phoneCam;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    MyOpenCvPipeline openCvPipeline;
    boolean useWebCamera;
    ArrayList<AprilTagDetection> detections = new ArrayList<>();


    public OpenCVWrapper(Telemetry tele, HardwareMap hwMap, boolean webCam) {
        telemetry = tele;
        hardwareMap = hwMap;
        useWebCamera = webCam;
    }

    public void init() {
        RobotLogger.dd(TAG, "init OpenCVWrapper");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //cone_tracker.setTelemetry(telemetry);
        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        RobotLogger.dd(TAG, "getIdentifier " + cameraMonitorViewId);

        if (!useWebCamera)
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        else
            phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        RobotLogger.dd(TAG, "OpenCvCameraFactory " + phoneCam.toString());

        // Connect to the camera
        phoneCam.openCameraDevice();
        // Use the OpenCVObjectDetector pipeline
        // processFrame() will be called to process the frame
        openCvPipeline = AprilTagDetectionPipeline.getAprilTagSingleInstance();
        assert(openCvPipeline != null);
        phoneCam.setPipeline(openCvPipeline);
        openCvPipeline.setTelemetry(telemetry);
    }

    public void start() {
        RobotLogger.dd(TAG, "start OpenCVWrapper");

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
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
        //openCvPipeline.stopPipeline();
        RobotLogger.dd(TAG, "to closeCameraDevice");
        phoneCam.closeCameraDevice();
        /*
        phoneCam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                RobotLogger.dd(TAG, "closeCameraDeviceAsync onClose");
            }

        });
         */
    }

    private ArrayList<AprilTagDetection> getLatestDetections() {
        return openCvPipeline.getLatestDetections();
    }

    // logic needed by auto client
    private ArrayList<AprilTagDetection> getDetectionsUpdate() {
        ArrayList<AprilTagDetection> latestDetections = openCvPipeline.getDetectionsUpdate();
        if (latestDetections != null && latestDetections.size() > 0)
            return latestDetections;
        else {
            RobotLogger.dd("", "not detecting anything, return the latest instead");
            return openCvPipeline.getLatestDetections();
        }
    }

    private int getAprilTagID() {
        ArrayList<AprilTagDetection> currentDetections = getDetectionsUpdate();
        int ret = -1;
        if(currentDetections != null && currentDetections.size() != 0) {
            for (AprilTagDetection tag : currentDetections) {
                RobotLogger.dd("", "detected num of tagIDs " + currentDetections.size()
                    + " tagID: " + tag.id);
                ret = tag.id;
                break;
            }
        }
        return ret;
    }

    public int getParkingNum() {
        int tagId = getAprilTagID();
        int parkingNum = 0;
        switch (tagId) {
            case 2:
                parkingNum = 1;
                break;
            case 1:
                parkingNum = 2;
                break;
            default:
                parkingNum = 0;  // default parking number???
        }
        RobotLogger.dd("", "parking number " + parkingNum);
        return parkingNum;
    }
    public void zoomCamera(int zoom) {
        //phoneCam.setZoom(zoom);
    }

    public OpenCvCamera getCamera() {
        return phoneCam;
    }
}
