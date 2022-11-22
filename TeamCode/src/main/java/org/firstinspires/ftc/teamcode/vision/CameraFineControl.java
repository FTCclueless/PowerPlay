package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

public class CameraFineControl {
    private String TAG = "CameraFineControl";

    ExposureControl myExposureControl;  // declare exposure control object
    long minExp;
    long maxExp;
    long curExp;            // exposure is duration, in time units specified

    GainControl myGainControl;      // declare gain control object
    int minGain;
    int maxGain;
    int curGain;
    boolean wasSetGainSuccessful;   // returned from setGain()

    boolean isAEPriorityOn = false;

    PtzControl myPtzControl;

    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;
    Telemetry telemetry;
    TfodCameraControlData lastTfodCameraControl;

    public CameraFineControl(VuforiaLocalizer vu,  Telemetry tele, TFObjectDetector tfod_) {
        vuforia = vu;
        tfod = tfod_;
        telemetry = tele;
        // Assign the exposure and gain control objects, to use their methods.
        myExposureControl = vuforia.getCamera().getControl(ExposureControl.class);
        myGainControl = vuforia.getCamera().getControl(GainControl.class);
        myPtzControl = null;
        lastTfodCameraControl = new TfodCameraControlData(0, 0, false, 0, 0);
    }

    public CameraFineControl(Telemetry tele, OpenCvWebcam webCam) {
        vuforia = null;
        tfod = null;
        telemetry = tele;
        myExposureControl = webCam.getExposureControl();
        myGainControl = webCam.getGainControl();
        myPtzControl = webCam.getPtzControl();
        lastTfodCameraControl = new TfodCameraControlData(0, 0, false, 0, 0);
    }

    private void getCameraFineControl() {
        // get webcam exposure limits
        minExp = myExposureControl.getMinExposure(TimeUnit.MILLISECONDS);
        maxExp = myExposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        // get webcam gain limits
        minGain = myGainControl.getMinGain();
        maxGain = myGainControl.getMaxGain();

        // Change mode to Manual, in order to control directly.
        // A non-default setting may persist in the camera, until changed again.
        myExposureControl.setMode(ExposureControl.Mode.Manual);

        // Retrieve from webcam its current exposure and gain values
        curExp = myExposureControl.getExposure(TimeUnit.MILLISECONDS);
        curGain = myGainControl.getGain();

        // display exposure mode and starting values to user
        telemetry.addLine("\nTouch Start arrow to control webcam Exposure and Gain");
        telemetry.addData("\nCurrent exposure mode", myExposureControl.getMode());
        telemetry.addData("Current exposure value", curExp);
        telemetry.addData("Current gain value", curGain);
        telemetry.update();
    }

    public void doCameraFineControlGamePad() {
        // manually adjust the webcam exposure & gain variables
        float changeExp = -gamepad1.left_stick_y;
        float changeGain = -gamepad1.right_stick_y;

        int changeExpInt = (int) (changeExp*2);     // was *5
        int changeGainInt = (int) (changeGain*2);   // was *5

        curExp += changeExpInt;
        curGain += changeGainInt;

        if (gamepad1.a) {           // AE Priority ON with green A
            myExposureControl.setAePriority(true);
            isAEPriorityOn = true;
        } else if (gamepad1.b) {    // AE Priority OFF with red B
            myExposureControl.setAePriority(false);
            isAEPriorityOn = false;
        }

        // ensure inputs are within webcam limits, if provided
        curExp = Math.max(curExp, minExp);
        curExp = Math.min(curExp, maxExp);
        curGain = Math.max(curGain, minGain);
        curGain = Math.min(curGain, maxGain);

        // update the webcam's settings
        myExposureControl.setExposure(curExp, TimeUnit.MILLISECONDS);
        wasSetGainSuccessful = myGainControl.setGain(curGain);
    }

    public void doCameraFineControl() {
        /* comment for now
        if (!VirtualConfig.tfodCameraControl.equal(lastTfodCameraControl)) {
            RobotLogger.dd(TAG, "camera control paramters changed, need to apply " + VirtualConfig.tfodCameraControl);
            if (tfod != null)
                tfod.setZoom(VirtualConfig.tfodCameraControl.tfodZoom, VirtualConfig.tfodCameraControl.tfodAspectRatio);
            else {
                if (myPtzControl != null) {
                    int maxZoom = myPtzControl.getMaxZoom();
                    int minZoom = myPtzControl.getMinZoom();
                    int currZoom = myPtzControl.getZoom();
                    RobotLogger.dd(TAG, "maxZoom: " + maxZoom + " minZoom: " + minZoom + " currZoom: " + currZoom);
                    int zoom = (int) VirtualConfig.tfodCameraControl.tfodZoom;
                    if (zoom >= minZoom && zoom <= maxZoom && zoom != currZoom) {
                        myPtzControl.setZoom(zoom);
                        RobotLogger.dd(TAG, "setZoom: " + zoom);
                    }
                }
            }

            curExp = VirtualConfig.tfodCameraControl.tfodCameraExposure;
            curGain = VirtualConfig.tfodCameraControl.tfodCameraGain;

            if (VirtualConfig.tfodCameraControl.tfodAutoExposurePriority) {  // AE Priority ON with green A
                myExposureControl.setAePriority(true);
                isAEPriorityOn = true;
            } else {    // AE Priority OFF with red B
                myExposureControl.setAePriority(false);
                isAEPriorityOn = false;
            }

            // ensure inputs are within webcam limits, if provided
            curExp = Math.max(curExp, minExp);
            curExp = Math.min(curExp, maxExp);
            curGain = Math.max(curGain, minGain);
            curGain = Math.min(curGain, maxGain);

            // update the webcam's settings
            myExposureControl.setExposure(curExp, TimeUnit.MILLISECONDS);
            wasSetGainSuccessful = myGainControl.setGain(curGain);

            lastTfodCameraControl.clone(VirtualConfig.tfodCameraControl);
            getCameraFineControl();
        }

         */
    }
}