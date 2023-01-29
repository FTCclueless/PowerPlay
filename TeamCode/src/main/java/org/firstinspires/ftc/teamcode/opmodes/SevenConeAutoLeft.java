package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Robot.STATE.DEPOSIT_AUTO;
import static org.firstinspires.ftc.teamcode.Robot.STATE.IDLE;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_GLOBAL;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_RELATIVE;
import static org.firstinspires.ftc.teamcode.Robot.STATE.SCORING_GLOBAL;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.vision.OpenCVWrapper;

@Autonomous(group = "Auto")
public class SevenConeAutoLeft extends LinearOpMode {
    public static final int cycles = 4;
    public static final int cycles2 = 2;
    public static int parkingNum = 0;
    public static final boolean lr = true; // Left : true | Right : false

    OpenCVWrapper openCVWrapper;

    double[] coneStackHeights = new double[]{5.4, 4.15, 3.0, 1.5, 0.0}; //5.65, 4.4, 2.75, 2.0, 0.5
    ButtonToggle toggleA = new ButtonToggle();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        DriveConstants.MAX_ACCEL = 58;

        openCVWrapper = new OpenCVWrapper(telemetry, hardwareMap, true);
        assert(openCVWrapper != null);

        // Signs
        int ySign = lr ? 1 : -1;

        Pose2d origin = new Pose2d(
                37,
                62 * ySign,
                lr ? Math.toRadians(90) : Math.toRadians(-90)
        );

        Pose2d toPose = new Pose2d(
                origin.getX()-2,
                20 * ySign,
                lr ? Math.toRadians(90) : Math.toRadians(-90)
        );

        Pose2d cyclePose = new Pose2d(
                47.1,
                13 * ySign,
                Math.toRadians(180)
        );

        Pose2d cyclePose2 = new Pose2d(
                -cyclePose.getX(),
                12.75,
                Math.toRadians(180)
        );

        robot.stayInPlacePose = cyclePose;

        TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(toPose.getX(), toPose.getY()))
                .splineTo(new Vector2d(cyclePose.getX(), cyclePose.getY()), Math.toRadians(0))
                .addDisplacementMarker(0, () -> {
                    robot.currentState = Robot.STATE.SCORING_GLOBAL;
                    robot.startScoringGlobal(
                            new Pose2d(toPose.getX(), toPose.getY()+10, toPose.getHeading()),
                            new Pose2d(23, -1.0 * ySign), // 23, -1
                            28.5);
                })
                .build();

        TrajectorySequence moveToOppositeSide = drive.trajectorySequenceBuilder(new Pose2d(cyclePose.getX(), cyclePose.getY(), Math.toRadians(180)))
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(cyclePose2.getX(), cyclePose2.getY()))
                .addDisplacementMarker(0, () -> {
                    robot.currentState = Robot.STATE.SCORING_GLOBAL;
                    robot.startScoringGlobal(
                            new Pose2d(0.0, cyclePose2.getY(), cyclePose2.getHeading()),
                            new Pose2d(-23, -1.0 * ySign),
                            28.5);
                })
                .build();

        drive.setPoseEstimate(origin);

        Trajectory[] park = new Trajectory[]{
                drive.trajectoryBuilder(cyclePose2).strafeTo(new Vector2d( // parking position 3
                        -13,
                        cyclePose.getY()
                )).build(),
                drive.trajectoryBuilder(cyclePose2).strafeTo(new Vector2d( // parking position 2
                        -34,
                        cyclePose.getY()
                )).build(),
                drive.trajectoryBuilder(cyclePose2).strafeTo(new Vector2d( // parking position 1
                        -59.5,
                        cyclePose.getY()
                )).build()
        };

        robot.resetEncoders();
        robot.claw.open();

        openCVWrapper.init();
        openCVWrapper.start();

        Log.e("camera setup", "");

        sleep(1000);
        robot.initPosition(true);

        Log.e("init position", "");

        while (opModeInInit()) {
            telemetry.setMsTransmissionInterval(50);

            boolean detected = false;

            parkingNum = openCVWrapper.getParkingNum();
            detected = true;

            if (toggleA.isClicked(gamepad1.a)) {
                robot.claw.close();
            }

            robot.update();

            if (detected) {
                telemetry.addLine("Tag of interest is in sight! ID: " + (parkingNum + 1));
            } else {
                telemetry.addLine("Could not find april tag! :(");
            }

            telemetry.update();
        }

        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)), to.end());
        drive.setPoseEstimate(origin);
        waitForStart();
        long startTime = System.currentTimeMillis();

        openCVWrapper.stop();

        robot.followTrajectorySequence(to, this);
        robot.updateStayInPlacePID = true;

        while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT_AUTO) {
            robot.update();
        }

        for (int i = 0; i < cycles; i++) {
            robot.currentState = INTAKE_GLOBAL;
            // TODO verify the x and y sign on this. It should not be like this
            robot.startIntakeGlobal(
                    to.end(),
                    new Pose2d(70,12 * ySign),
                    coneStackHeights[i]
            );

            while (robot.currentState == INTAKE_GLOBAL) {
                robot.update();
            }

            robot.startScoringGlobal(
                    new Pose2d(to.end().getX(), to.end().getY(), to.end().getHeading()),
                    new Pose2d(23,-1.0 * ySign),
                    29);

            while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT_AUTO) {
                robot.update();
            }
        }

        // hold onto last cone

        robot.currentState = INTAKE_GLOBAL;
        // TODO verify the x and y sign on this. It should not be like this
        robot.startIntakeGlobal(
                to.end(),
                new Pose2d(70,12 * ySign),
                coneStackHeights[4]
        );

        while (robot.currentState == INTAKE_GLOBAL) {
            robot.update();
        }

        while (!robot.outtake.isInPosition(20)) {
            robot.update();
        }

        robot.updateStayInPlacePID = false;
        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)), moveToOppositeSide.end());

        // going over to other side of field

        robot.followTrajectorySequence(moveToOppositeSide, this);
        robot.stayInPlacePose = cyclePose2;
        robot.updateStayInPlacePID = true;
        robot.outtake.actuation.level();

        for (int i = 0; i < cycles2; i++) {
            robot.currentState = INTAKE_GLOBAL;
            robot.startIntakeGlobal(
                    moveToOppositeSide.end(),
                    new Pose2d(-71,12 * ySign),
                    coneStackHeights[i]
            );

            while (robot.currentState == INTAKE_GLOBAL) {
                robot.update();
            }

            robot.startScoringGlobal(
                    new Pose2d(moveToOppositeSide.end().getX(), moveToOppositeSide.end().getY(), moveToOppositeSide.end().getHeading()),
                    new Pose2d(-23,-1.0 * ySign),
                    29);

            while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT_AUTO) {
                robot.update();
            }
        }

        robot.updateStayInPlacePID = false;
        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)), park[parkingNum].end());

        // parking

        robot.claw.park();
        robot.outtake.actuation.level();
        robot.update();

        robot.followTrajectory(park[parkingNum], this, startTime);

        Storage.autoEndPose = drive.getPoseEstimate();
        Storage.isBlue = true;
    }
}