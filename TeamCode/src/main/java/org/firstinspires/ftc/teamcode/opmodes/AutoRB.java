package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

// HEY HEY ! DON'T READ THIS CODE! ITS FOR EMERGENCY
@Autonomous(group = "Test")
public class AutoRB extends LinearOpMode {
    public static int parkingNum = 0;
    public static final boolean lr = false; // Left : true | Right : false
    public static final boolean tb = false; // Top : true | Bottom : false
    public static OpenCvCamera camera;
    public AprilTagDetectionPipeline atdp = new AprilTagDetectionPipeline(
            0.166, // Size of april tag in meters
            // These 4 values are calibration for the C920 webcam (800x448)
            578.272,
            578.272,
            402.145,
            221.506
    );
    //    double[] coneStackHeights = new double[] {5.74, 4.305, 2.87, 1.435, 0.0};
    double[] coneStackHeights = new double[]{5.85, 4.1, 2.4, 1.435, 0.0};
    ButtonToggle toggleA = new ButtonToggle();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(atdp);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT); // need to change for phone back camera
            }

            @Override
            public void onError(int errorCode) {
                Log.e("error with vision", "");
            }
        });

        // Signs
        int xSign = tb ? 1 : -1;
        int ySign = lr ? 1 : -1;

        Pose2d origin = new Pose2d(
                36 * xSign,
                60 * ySign,
                Math.PI / 2 * (!lr ? -1 : 1)
        );

        drive.setPoseEstimate(origin);

        Pose2d intakePose = new Pose2d(
                55 * xSign,
                12 * ySign,
                tb ? 0 : Math.PI
        );

        Pose2d depositPose = new Pose2d(
                38 * xSign,
                12 * ySign,
                tb ? 0 : Math.PI
        );

        drive.setPoseEstimate(origin); // FIXME is this needed?
        TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
                // Move forward extra in order to bump away the signal cone
                .strafeTo(new Vector2d(origin.getX(), origin.getY() - (54 * ySign)))
                .lineToLinearHeading(new Pose2d(depositPose.getX() + (4 * xSign), depositPose.getY(), depositPose.getHeading()))
                .build();

        // TODO talk to hudson about this weird heading stuff
        TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(new Pose2d(intakePose.getX() - (3 * xSign), intakePose.getY(), intakePose.getHeading()))
                .lineToConstantHeading(new Vector2d(depositPose.getX(), depositPose.getY()))
                .build();

        TrajectorySequence toIntake = drive.trajectorySequenceBuilder(new Pose2d(depositPose.getX() + (2 * xSign), depositPose.getY(), depositPose.getHeading()))
                .lineToConstantHeading(new Vector2d(intakePose.getX(), intakePose.getY()))
                .build();

        // TODO clean this up a little? Kinda lookin a little bad
        Trajectory[] park = new Trajectory[]{
                drive.trajectoryBuilder(toDeposit.end()).strafeTo(new Vector2d(
                        origin.getX() + (23.5 + (tb ? 0 : 1.5)),
                        depositPose.getY()
                )).build(),
                drive.trajectoryBuilder(toDeposit.end()).strafeTo(new Vector2d(
                        origin.getX() - (2.001 * ySign),
                        depositPose.getY()
                )).build(),
                drive.trajectoryBuilder(toDeposit.end()).strafeTo(new Vector2d(
                        origin.getX() - (27),
                        depositPose.getY()
                )).build()
        };

        robot.resetEncoders();
        robot.claw.open();

        while (opModeInInit()) {
            telemetry.setMsTransmissionInterval(50);

            boolean detected = false;
            ArrayList<AprilTagDetection> currentDetections = atdp.getDetectionsUpdate();

            if (currentDetections != null && currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections) {
                    switch (tag.id) {
                        case 2:
                            parkingNum = 1;
                            break;
                        case 1:
                            parkingNum = 2;
                            break;
                        default:
                            parkingNum = tag.id;
                    }
                    detected = true;
                }
            }

            robot.actuation.level();
            robot.outtake.extension.retractExtension();
            if (toggleA.isClicked(gamepad1.a)) {
                robot.claw.close();
            }

            robot.update();

            if (detected) {
                telemetry.addLine(String.format("Tag of interest is in sight! ID: %d", parkingNum + 1));
            } else {
                telemetry.addLine("Could not find april tag! :(");
            }

            telemetry.update();
        }

        waitForStart();

        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(2.5, 2.5, Math.toRadians(5)), to.end());
        robot.followTrajectorySequence(to, this);
        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(2.5, 2.5, Math.toRadians(5)), toDeposit.end());
        robot.startScoringGlobal(toDeposit.end(), new Pose2d(26.25 * xSign,0),28.3, xSign * ySign); // 36
        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)), park[parkingNum].end());
        robot.followTrajectory(park[parkingNum], this);
    }
}