package org.firstinspires.ftc.teamcode.vision.opmode;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;
import org.firstinspires.ftc.teamcode.vision.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.vision.TFODWrapper;

//@Disabled
@TeleOp(name = "VisionWrapperTest", group = "Concept")
public class VisionWrapperTest extends LinearOpMode {
    TFODWrapper tfodWrapper;
    OpenCVWrapper openCVWrapper;
    String[]  labels = {
        "1 base",
        "2 top"
    };

    private String TAG = "VisionWrapperTest";
    boolean useWebCamera = false;
    boolean runTFOD = false;
    boolean runAprilTag = true;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        tfodWrapper = new TFODWrapper("/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite", labels, useWebCamera, telemetry, hardwareMap);
        tfodWrapper.setOpMode(this);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        long start_time = SystemClock.elapsedRealtime();
        waitForStart();
        int done;
        if (opModeIsActive()) {
            String printStr;
            while (opModeIsActive()) {
                if (runTFOD) {
                    tfodWrapper.init();
                    FtcDashboard.getInstance().startCameraStream(tfodWrapper.getTFOD(), 0);

                    printStr = "tfod started, switching takes: " + String.valueOf(SystemClock.elapsedRealtime() - start_time);
                    RobotLogger.dd(TAG, printStr);
                    telemetry.addLine(printStr);
                    telemetry.update();

                    done = 0;
                    while (done < 20) {
                        tfodWrapper.start();
                        done++;
                        SafeSleep.sleep_milliseconds(this, 50);
                    }

                    SafeSleep.sleep_milliseconds(this, 5000);
                    RobotLogger.dd(TAG, "to stop tfod");
                    start_time = SystemClock.elapsedRealtime();
                    tfodWrapper.stop();
                    FtcDashboard.getInstance().stopCameraStream();

                }
                if (runAprilTag) {
                    openCVWrapper = new OpenCVWrapper(telemetry, hardwareMap, useWebCamera);

                    openCVWrapper.init();
                    openCVWrapper.start();
                    //FtcDashboard.getInstance().startCameraStream(openCVWrapper.getCamera(), 0);

                    printStr = "opencv started, switching takes: " + String.valueOf(SystemClock.elapsedRealtime() - start_time);
                    RobotLogger.dd(TAG, printStr);
                    telemetry.addLine(printStr);
                    telemetry.update();

                    SafeSleep.sleep_milliseconds(this, 500);
                    done = 0;
                    while (done < 500) {
                        openCVWrapper.getParkingNum();
                        done++;
                        SafeSleep.sleep_milliseconds(this, 10);
                    }

                    SafeSleep.sleep_milliseconds(this, 5000);
                    RobotLogger.dd(TAG, "to stop opencv");
                    start_time = SystemClock.elapsedRealtime();
                    //FtcDashboard.getInstance().stopCameraStream();
                    openCVWrapper.stop();
                }
                //SafeSleep.sleep_milliseconds(this,5000);
            }
        }
    }
}
