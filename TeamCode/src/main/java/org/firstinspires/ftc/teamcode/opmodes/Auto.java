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
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.OpenCVWrapper;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Auto extends LinearOpMode {
    public final int cycles = 5;
    public int parkingNum = 0;
    public boolean lr = true; // Left : true | Right : false
    public boolean tb = true; // Top : true | Bottom : false

    public OpenCVWrapper openCVWrapper;

    public double[] coneStackHeights = new double[]{5.0, 4.0, 2.6, 1.435, 0.0};
    public ButtonToggle toggleA = new ButtonToggle();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        Storage.isTeleop = false;

        openCVWrapper = new OpenCVWrapper(telemetry, hardwareMap, true);
        assert (openCVWrapper != null);

        // Signs
        int xSign = tb ? 1 : -1;
        int ySign = lr ? 1 : -1;

        Pose2d origin = new Pose2d(
            36 * xSign,
            60 * ySign,
            Math.PI / 2 * ySign
        );

        drive.setPoseEstimate(origin);

        Pose2d intakePose = new Pose2d(
            49.5 * xSign,
            13.5 * ySign,
            tb ? Math.PI : 0
        );

        Pose2d depositPose = new Pose2d(
            38 * xSign,
            13.5 * ySign,
            tb ? Math.PI : 0
        );

        drive.setPoseEstimate(origin); // FIXME is this needed?
        TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
            // Move forward extra in order to bump away the signal cone
            .strafeTo(new Vector2d(origin.getX(), origin.getY() - (54 * ySign)))
            .addDisplacementMarker(12, () -> {
                robot.ySign = xSign * ySign;
                robot.currentState = Robot.STATE.SCORING_GLOBAL;
                robot.startScoringGlobal(new Pose2d(depositPose.getX() + (5 * xSign), depositPose.getY(), depositPose.getHeading()), new Pose2d(28.8 * xSign, 0.25 * ySign), 27.7, xSign * ySign); // 36
            })
            .lineToLinearHeading(new Pose2d(depositPose.getX() + (5 * xSign), depositPose.getY(), depositPose.getHeading()))
            .build();

        TrajectorySequence toDeposit = drive.trajectorySequenceBuilder(new Pose2d(intakePose.getX() - (3 * xSign), intakePose.getY(), intakePose.getHeading()))
            .lineToConstantHeading(new Vector2d(depositPose.getX(), depositPose.getY()))
            .build();

        TrajectorySequence toIntake = drive.trajectorySequenceBuilder(new Pose2d(depositPose.getX() + (2 * xSign), depositPose.getY(), depositPose.getHeading()))
            .lineToConstantHeading(new Vector2d(intakePose.getX(), intakePose.getY()))
            .build();

        Trajectory[] park = new Trajectory[]{
            drive.trajectoryBuilder(toDeposit.end()).strafeTo(new Vector2d(
                origin.getX() + (23.5 + (tb ? 0 : 1.5)),
                depositPose.getY()
            )).build(),
            drive.trajectoryBuilder(toDeposit.end()).strafeTo(new Vector2d(
                origin.getX() - (2.000001 * ySign),
                depositPose.getY()
            )).build(),
            drive.trajectoryBuilder(toDeposit.end()).strafeTo(new Vector2d(
                origin.getX() - (27),
                depositPose.getY()
            )).build()
        };

        robot.resetEncoders();
        robot.claw.open();

        openCVWrapper.init();
        openCVWrapper.start();

        Log.e("camera setup", "");

        sleep(2000);
        robot.initPosition();

        Log.e("init position", "");

        while (opModeInInit()) {
            telemetry.setMsTransmissionInterval(50);

            boolean detected = false;

            parkingNum = openCVWrapper.getParkingNum();
            detected = true;  //should we always set to true ??? It is only used to send telemetry anyways

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

        drive.setPoseEstimate(origin);
        waitForStart();

        openCVWrapper.stop();

        // preload

        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(2.5, 2.5, Math.toRadians(5)), to.end());

        robot.followTrajectorySequence(to, this);

        robot.startScoringGlobal(new Pose2d(to.end().getX() + (2.75 * xSign), to.end().getY(), to.end().getHeading()), new Pose2d(24 * xSign, -1 * ySign), 26, xSign * ySign); // 36
        while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT) {
            robot.update();
        }

        for (int i = 0; i < cycles; i++) {
            robot.drivetrain.setBreakFollowingThresholds(new Pose2d(2.5, 2.5, Math.toRadians(5)), toIntake.end());

            robot.currentState = INTAKE_GLOBAL;
            robot.startIntakeGlobal(
                toIntake.end(),
                new Pose2d(70 * xSign, 12 * ySign),
                coneStackHeights[i]
            );

            robot.followTrajectorySequence(toIntake, this);

            while (robot.currentState == INTAKE_GLOBAL) {
                robot.update();
            }

            robot.drivetrain.setBreakFollowingThresholds(new Pose2d(2.5, 2.5, Math.toRadians(5)), toDeposit.end());

            robot.startScoringGlobal(new Pose2d(toDeposit.end().getX() + (2.75 * xSign), toDeposit.end().getY(), toDeposit.end().getHeading()), new Pose2d(24 * xSign, -1 * ySign), 26.1, xSign * ySign); // 36
            robot.followTrajectorySequence(toDeposit, this);
            while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT) {
                robot.update();
            }
        }

        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)), park[parkingNum].end());

        robot.followTrajectory(park[parkingNum], this);

        long clawStart = System.currentTimeMillis();
        robot.claw.park();

        while (System.currentTimeMillis() - clawStart <= 300) {
            robot.claw.park();
            robot.claw.update();
        }

        Storage.autoEndPose = drive.getPoseEstimate();
        Storage.isBlue = true;
    }
}