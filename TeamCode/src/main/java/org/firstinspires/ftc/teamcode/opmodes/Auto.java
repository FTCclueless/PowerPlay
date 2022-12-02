package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Robot.STATE.DEPOSIT;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_GLOBAL;
import static org.firstinspires.ftc.teamcode.Robot.STATE.SCORING_GLOBAL;

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
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "Test")
public class Auto extends LinearOpMode {
    public static final int cycles = 5;
    public static int parkingNum = 0;
    public static final boolean lr = true; // Left : true | Right : false
    public static final boolean tb = true; // Top : true | Bottom : false
    public static final double cycleBack = 6; // Once robot gets to cycle position how much it moves backwards
    public static final double cycleY = 48; // Turning can give an offset (+ cone location)
    public static OpenCvCamera camera;
    public static final AprilTagDetectionPipeline atdp = new AprilTagDetectionPipeline(
        0.166, // Size of april tag in meters
        // These 4 values are calibration for the C920 webcam (800x448)
        578.272,
        578.272,
        402.145,
        221.506
    );
    public static final double coneStackAdditionalHeight = 1.38;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        camera.setPipeline(atdp);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT); // need to change for phone back camera
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                Log.e("error with vision", "");
//            }
//        });

        // Signs
        int xSign = tb ? 1 : -1;
        int ySign = lr ? 1 : -1;

        Pose2d origin = new Pose2d(
            36 * xSign,
            60 * ySign,
            Math.PI / 2 * (!lr ? -1 : 1)
        );

        drive.setPoseEstimate(origin);

        double heading = Math.toRadians(tb ? 0 : 180);

        Pose2d intakePose = new Pose2d(
                55,
                12
        );

        Pose2d depositPose = new Pose2d(
                32,
                12
        );

        drive.setPoseEstimate(origin); // FIXME is this needed?
        TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
                // Move forward extra in order to bump away the signal cone
                .strafeTo(new Vector2d(origin.getX(), origin.getY() - (52 * ySign)))
                .lineToLinearHeading(new Pose2d(depositPose.getX(),depositPose.getY(), depositPose.getHeading()))
                .build();

        TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(new Pose2d(intakePose.getX() - 3, intakePose.getY()))
                .splineToConstantHeading(new Vector2d(depositPose.getX(),depositPose.getY()), heading)
                .build();

        TrajectorySequence toIntake = drive.trajectorySequenceBuilder(new Pose2d(depositPose.getX() + 2, depositPose.getY()))
                .splineToConstantHeading(new Vector2d(intakePose.getX(), intakePose.getY()), heading)
                .build();

        Trajectory[] park = new Trajectory[]{
            drive.trajectoryBuilder(toDeposit.end()).strafeTo(new Vector2d(origin.getX() + (23.5 + (tb ? 0 : 1.5)), origin.getY() - (cycleY * ySign))).build(),
            drive.trajectoryBuilder(toDeposit.end()).strafeTo(new Vector2d(origin.getX() - (2 * ySign), origin.getY() - (cycleY * ySign))).build(),
            drive.trajectoryBuilder(toDeposit.end()).strafeTo(new Vector2d(origin.getX() - (25), origin.getY() - (cycleY * ySign))).build()
        };

        robot.resetEncoders();

        while (opModeInInit()) {
            telemetry.setMsTransmissionInterval(50);

//            boolean detected = false;
//            ArrayList<AprilTagDetection> currentDetections = atdp.getLatestDetections();
//
//            if (currentDetections.size() != 0) {
//                for (AprilTagDetection tag : currentDetections) {
//                    switch (tag.id) {
//                        case 2:
//                            parkingNum = 1;
//                            break;
//                        case 1:
//                            parkingNum = 2;
//                            break;
//                        default:
//                            parkingNum = tag.id;
//                    }
//                    detected = true;
//                }
//            }

            robot.actuation.level();
            robot.outtake.extension.retractExtension();
            if (robot.outtake.extension.isInPosition(5)) {
                robot.claw.intake();
            }
            robot.update();

//            if (detected) {
//                telemetry.addLine(String.format("Tag of interest is in sight! ID: %d", parkingNum + 1));
//            } else {
//                telemetry.addLine("Could not find april tag! :(");
//            }
//
//            telemetry.update();
        }

        waitForStart();

        robot.followTrajectorySequence(to, this);

        for (int i = 0; i < cycles; i++) {
            robot.drivetrain.setBreakFollowingThresholds(new Pose2d(2.5, 2.5, Math.toRadians(5)), toIntake.end());

            robot.currentState = Robot.STATE.RETRACT;
            robot.startIntakeGlobal(
                toIntake.end(),
                new Pose2d((72 - 4) * xSign,12 * ySign),
                coneStackAdditionalHeight * (4 - i)
            );

            robot.followTrajectorySequence(toIntake, this);

            while (robot.currentState == INTAKE_GLOBAL) {
                robot.update();
            }

            robot.drivetrain.setBreakFollowingThresholds(new Pose2d(2.5, 2.5, Math.toRadians(5)), toDeposit.end());

            robot.startScoringGlobal(toDeposit.end(), new Pose2d(26 * xSign,0),28.5, ySign); // 36
            robot.followTrajectorySequence(toDeposit, this);
            robot.update();
            while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT) {
                robot.update();
            }

            while (robot.outtake.slides.currentSlidesLength > 15) {
                robot.update();
            }
        }
        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)), park[parkingNum].end());

        robot.followTrajectory(park[parkingNum], this);
    }
}