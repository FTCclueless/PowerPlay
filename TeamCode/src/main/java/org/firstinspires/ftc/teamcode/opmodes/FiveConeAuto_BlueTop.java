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

@Autonomous(group = "Test")
public class FiveConeAuto_BlueTop extends LinearOpMode {
    public static final int cycles = 5;
    public static int parkingNum = 0;
    public static final boolean lr = true; // Left : true | Right : false
    public static final boolean tb = true; // Top : true | Bottom : false
    //public static final double cycleBack = 6; // Once robot gets to cycle position how much it moves backwards
    //public static final double cycleY = 48; // Turning can give an offset (+ cone location)

    OpenCVWrapper openCVWrapper;

    double[] coneStackHeights = new double[]{4.15, 3.3, 2.4, 1.435, 0.0};
    ButtonToggle toggleA = new ButtonToggle();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;

        openCVWrapper = new OpenCVWrapper(telemetry, hardwareMap, true);
        assert(openCVWrapper != null);

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
            53.5 * xSign,
            9.5 * ySign,
            tb ? 0 : Math.PI
        );

        Pose2d depositPose = new Pose2d(
            38 * xSign,
            9.5 * ySign,
            tb ? 0 : Math.PI
        );

        drive.setPoseEstimate(origin); // FIXME is this needed?
        TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
            // Move forward extra in order to bump away the signal cone
            .strafeTo(new Vector2d(origin.getX(), origin.getY() - (54 * ySign)))
            .addDisplacementMarker(12, () -> {
                robot.ySign = xSign * ySign;
                robot.currentState = Robot.STATE.SCORING_GLOBAL;
                robot.startScoringGlobal(new Pose2d(depositPose.getX() + (5 * xSign), depositPose.getY(), depositPose.getHeading()), new Pose2d(28.8 * xSign,0.25 * ySign),27.7, xSign * ySign); // 36
            })
            .lineToLinearHeading(new Pose2d(depositPose.getX() + (5 * xSign), depositPose.getY(), depositPose.getHeading()))
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

        while (opModeInInit()) {
            telemetry.setMsTransmissionInterval(50);

            boolean detected = false;

            /////
            parkingNum = openCVWrapper.getParkingNum();
            detected = true;  //should we always set to true ??? It is only used to send telemetry anyways
            ///////////

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

        openCVWrapper.stop();

        // preload

        robot.currentState = Robot.STATE.RETRACT;

        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(2.5, 2.5, Math.toRadians(5)), to.end());

        robot.followTrajectorySequence(to, this);

        robot.startScoringGlobal(to.end(), new Pose2d(27.2 * xSign,0.25 * ySign),25.9, xSign * ySign); // 36
        while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT) {
            robot.update();
        }

        while (robot.outtake.slides.currentSlidesLength > 15) {
            robot.update();
        }

        for (int i = 0; i < cycles; i++) {
            robot.drivetrain.setBreakFollowingThresholds(new Pose2d(2.5, 2.5, Math.toRadians(5)), toIntake.end());

            robot.currentState = Robot.STATE.RETRACT;
            // TODO verify the x and y sign on this. It should not be like this
            robot.startIntakeGlobal(
                    toIntake.end(),
                    new Pose2d((72 - 4) * xSign,10.0 * ySign),
                    coneStackHeights[i]
            );

            robot.followTrajectorySequence(toIntake, this);

            while (robot.currentState == INTAKE_GLOBAL) {
                robot.update();
            }

            robot.drivetrain.setBreakFollowingThresholds(new Pose2d(2.5, 2.5, Math.toRadians(5)), toDeposit.end());

            robot.startScoringGlobal(toDeposit.end(), new Pose2d(27.5 * xSign,0),26.65, xSign * ySign); // 36
            robot.followTrajectorySequence(toDeposit, this);
            while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT) {
                robot.update();
            }

            while (robot.outtake.slides.currentSlidesLength > 15) {
                robot.update();
            }
        }
        // FIXME this could be a little bit stricter
        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)), park[parkingNum].end());

        robot.followTrajectory(park[parkingNum], this);

        Storage.autoEndPose = drive.getPoseEstimate();
    }
}