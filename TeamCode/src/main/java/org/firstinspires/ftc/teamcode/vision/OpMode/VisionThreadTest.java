package org.firstinspires.ftc.teamcode.vision.opmode;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.OpenCVWrapper;
import org.firstinspires.ftc.teamcode.vision.TFODWrapper;
import org.firstinspires.ftc.teamcode.vision.VisionThread;

import java.util.List;

@Disabled
@Config
@TeleOp(name = "VisionThreadTest", group = "Vision")
public class VisionThreadTest extends LinearOpMode {
    VisionThread tfodThread;
    String[]  labels = {
            "1 base",
            "2 top"
    };
    public static boolean enableCameraControl = false;
    public static boolean useWebCamera = true;
    private String TAG = "VisionThreadTest";
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        tfodThread = new VisionThread("/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite", labels, useWebCamera, telemetry, hardwareMap);
        tfodThread.setOpMode(this);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        FtcDashboard.getInstance().startCameraStream(tfodThread.getTFOD(), 0);

        tfodThread.start();  // TFOD can be started before button press;

        waitForStart();
        boolean done = false;
        while (opModeIsActive() && !done) {
            tfodThread.setCameraFindControlFlag(enableCameraControl);

            SafeSleep.sleep_milliseconds(this, 500);
            List<Recognition> updatedRecognitions = tfodThread.getDetectionsUpdate();
            if (updatedRecognitions != null && updatedRecognitions.size() != 0) {
                RobotLogger.dd(TAG, "TFOD detected: " + updatedRecognitions.size());
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());

                    telemetry.addData("", " ");
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                }
                telemetry.update();
                done = true;
            }
        }
        RobotLogger.dd(TAG, "to stop tfodThread");
        tfodThread.stopRunning();
        tfodThread.interrupt();
        FtcDashboard.getInstance().stopCameraStream();
        /*
        try {
            tfodThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        } */
    }
}
