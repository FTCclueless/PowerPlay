package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;
import org.firstinspires.ftc.teamcode.vision.ConeTracker;

@TeleOp(name = "VisionWrapperTest", group = "Concept")
public class VisionWrapperTest extends LinearOpMode {
    TFODWrapper tfodWrapper;
    OpenCVWrapper openCVWrapper;
    String[]  labels = {
        "1 base",
        "2 top"
    };

    private String TAG = "VisionWrapperTest";
    @Override
    public void runOpMode() {
        tfodWrapper = new TFODWrapper("/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite", labels, false, telemetry, hardwareMap);
        openCVWrapper = new OpenCVWrapper(telemetry, hardwareMap, new ConeTracker(), false);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            //FtcDashboard.getInstance().startCameraStream(vuforia, 0);
            while (opModeIsActive()) {
                tfodWrapper.init();
                RobotLogger.dd(TAG, "tfod started");
                int done = 0;
                while (done < 20) {
                    tfodWrapper.start();
                    done ++;
                    SafeSleep.sleep_milliseconds(this, 50);
                }

                SafeSleep.sleep_milliseconds(this, 5000);
                RobotLogger.dd(TAG, "to stop tfod");
                tfodWrapper.stop();

                openCVWrapper.init();
                openCVWrapper.start();
                RobotLogger.dd(TAG, "opencv started");
                SafeSleep.sleep_milliseconds(this,5000);
                RobotLogger.dd(TAG, "to stop opencv");
                openCVWrapper.stop();
                //SafeSleep.sleep_milliseconds(this,5000);

            }
        }
    }
}
