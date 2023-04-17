package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.vision.PoleDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Pose2d;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@TeleOp
public class VisionTest extends LinearOpMode {
    OpenCvCamera webcam;
    public static double min = 0;
    public static double max = 100;

    @Override
    public void runOpMode()
    {
        Robot robot = new Robot(hardwareMap);
        robot.resetEncoders();
        robot.outtake.turret.zeroPower = true;
        robot.drivetrain.setPoseEstimate(new Pose2d(48, -12,0));

        Log.e("starting init process", "");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        PoleDetectionPipeline visionPipeline = new PoleDetectionPipeline(robot, robot.drivetrain, robot.outtake);

        webcam.setPipeline(visionPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        while (opModeInInit()) {
            updateTelemetry();
            robot.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            robot.update();

            Canvas fieldOverlay = TelemetryUtil.packet.fieldOverlay();
            DashboardUtil.drawPole(fieldOverlay, visionPipeline.globalPolePose);

            Log.e("visionPipeline.globalPolePose.x", visionPipeline.globalPolePose.x + "");
            updateTelemetry();

            if(gamepad1.a)
            {
                webcam.stopStreaming();
                webcam.closeCameraDevice();
            }
        }
    }

    public void updateTelemetry() {
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.update();
    }
}
