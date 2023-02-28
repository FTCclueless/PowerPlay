package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.ArrayList;
import java.util.List;


public class TFODWrapper {
    String TFOD_MODEL_FILE;
    boolean useWebCam;
    String[] LABELS;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    boolean cameraFineControlEnabled;

    private static String TAG = "TFODWrapper";
    private static final String VUFORIA_KEY =
            "ARjSEzX/////AAABmTyfc/uSOUjluYpQyDMk15tX0Mf3zESzZKo6V7Y0O/qtPvPQOVben+DaABjfl4m5YNOhGW1HuHywuYGMHpJ5/uXY6L8Mu93OdlOYwwVzeYBhHZx9le+rUMr7NtQO/zWEHajiZ6Jmx7K+A+UmRZMpCmr//dMQdlcuyHmPagFERkl4fdP0UKsRxANaHpwfQcY3npBkmgE8XsmK4zuFEmzfN2/FV0Cns/tiTfXtx1WaFD0YWYfkTHRyNwhmuBxY6MXNmaG8VlLwJcoanBFmor2PVBaRYZ9pnJ4TJU5w25h1lAFAFPbLTz1RT/UB3sHT5CeG0bMyM4mTYLi9SHPOUQjmIomxp9D7R39j8g5G7hiKr2JP";  //Variable Place--Remember to insert key here
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private LinearOpMode opMode;
    private CameraFineControl cameraFineControl;
    private final Object detectionsUpdateSync = new Object();
    List<Recognition> detectionsUpdate = new ArrayList<>();
    private List<Recognition> detections = new ArrayList<>();

    public void setCameraFindControlFlag(boolean flag) {
        cameraFineControlEnabled = flag;
    }
    public boolean getCameraFindControlFlag() {
        return cameraFineControlEnabled;
    }
    public TFODWrapper(String model_file, String[] labels, boolean webCamera, Telemetry tele, HardwareMap hwMap) {
        TFOD_MODEL_FILE  = model_file;
        useWebCam = webCamera;
        LABELS = labels;
        telemetry = tele;
        hardwareMap = hwMap;
        cameraFineControlEnabled = false;
    }
    public void init() {
        RobotLogger.dd(TAG, "init TFODWrapper");
        initVuforia();
        initTfod();
        telemetry.addData(">", "TFOD initialized");
        RobotLogger.dd(TAG, "Vuforia/Tfod initialized");
        telemetry.update();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();//cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        if (!useWebCam)
            parameters.cameraDirection = CameraDirection.BACK;
        else
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
//        tfod.loadModelFromAsset(TFOD_MODEL_FILE, LABELS);
         tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
            RobotLogger.dd(TAG, "camera name: " + vuforia.getCameraName().toString());
            if (useWebCam) {
                cameraFineControl = new CameraFineControl(vuforia, telemetry, tfod);
            }
        }
    }
    public void setOpMode(LinearOpMode opmode)
    {
        opMode = opmode;
    }
    public List<Recognition> getLatestDetections()
    {
        return detections;
    }

    public List<Recognition> getDetectionsUpdate()
    {
        synchronized (detectionsUpdateSync)
        {
            List<Recognition> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }
    public TFObjectDetector getTFOD() {
        return tfod;
    }
    public void start() {
        RobotLogger.dd(TAG, "run TF object detection");

        if (tfod == null) return;
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.

        detections = tfod.getUpdatedRecognitions();
        synchronized (detectionsUpdateSync)
        {
            detectionsUpdate = detections;
        }

        if (detections != null) {
            RobotLogger.dd(TAG, "TFOD detected: " + detections.size());

            telemetry.addData("# Objects Detected", detections.size());
/*
            // step through the list of recognitions and display image position/size information for each one
            // Note: "Image number" refers to the randomized image orientation/number
            for (Recognition recognition : detections) {
                double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
            }
 */
        }
        if (useWebCam && cameraFineControlEnabled) {
            cameraFineControl.doCameraFineControl();
        }
        telemetry.update();
    }

    public void stop() {
        RobotLogger.dd(TAG, "stop TFODWrapper");
        if (tfod != null) {
            tfod.shutdown();
        }
        //if (Vuforia.isInitialized())
        //    Vuforia.deinit();
        vuforia.close();
        vuforia = null;
    }

}
