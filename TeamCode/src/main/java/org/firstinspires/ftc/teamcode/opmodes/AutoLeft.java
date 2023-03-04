package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Robot.STATE.DEPOSIT_AUTO;
import static org.firstinspires.ftc.teamcode.Robot.STATE.IDLE;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_GLOBAL;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_RELATIVE;
import static org.firstinspires.ftc.teamcode.Robot.STATE.SCORING_GLOBAL;
import static org.firstinspires.ftc.teamcode.Robot.STATE.WAIT_FOR_START_SCORING;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.vision.OpenCVWrapper;

@Autonomous(group = "Auto")
public class AutoLeft extends LinearOpMode {
    public static final int cycles = 5;
    public static int parkingNum = 0;
    public static final boolean lr = true; // Left : true | Right : false

    OpenCVWrapper openCVWrapper;

    double[] coneStackHeights = new double[]{4.5, 3.5, 2.6, 1.65, 0.5}; //5.65, 4.4, 2.75, 2.0, 0.5
    ButtonToggle toggleA = new ButtonToggle();
    double[] timeToPark = new double[]{28000, 29000, 28000};

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        robot.currentState = IDLE;

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
                14 * ySign,
                Math.toRadians(180)
        );

        robot.stayInPlacePose = cyclePose;

        TrajectorySequence toPreload = drive.trajectorySequenceBuilder(origin)
                .lineToConstantHeading(new Vector2d(toPose.getX(), toPose.getY()))
                .addDisplacementMarker(5, () -> {
                    robot.currentState = Robot.STATE.SCORING_GLOBAL;
                    robot.startScoringGlobal(
                            new Pose2d(toPose.getX(), toPose.getY(), toPose.getHeading()),
                            new Pose2d(27.0, 0.0 * ySign),
                            26.75);
                })
                .build();

        TrajectorySequence toCycle = drive.trajectorySequenceBuilder(toPreload.end())
                .setReversed(true)
                .splineTo(new Vector2d(cyclePose.getX(), cyclePose.getY()), Math.toRadians(0))
                .build();

        drive.setPoseEstimate(origin);

        Trajectory[] park = new Trajectory[]{
                drive.trajectoryBuilder(cyclePose).strafeTo(new Vector2d( // parking position 1
                        59.5,
                        cyclePose.getY()
                )).build(),
                drive.trajectoryBuilder(cyclePose).strafeTo(new Vector2d( // parking position 2
                        34,
                        cyclePose.getY()
                )).build(),
                drive.trajectoryBuilder(cyclePose).strafeTo(new Vector2d( // parking position 3
                        13,
                        cyclePose.getY()
                )).build()
        };

        robot.resetEncoders();

        robot.initPosition(true);

        openCVWrapper.init();
        openCVWrapper.start();

        Log.e("camera setup", "");

        sleep(2000);

        Log.e("init position", "");

        robot.claw.init();
        robot.outtake.actuation.init();

        while (opModeInInit()) {
            telemetry.setMsTransmissionInterval(50);

            boolean detected = false;

            parkingNum = openCVWrapper.getParkingNum();
            detected = true;

            if (toggleA.isClicked(gamepad1.a)) {
                robot.claw.initClose();
            }

            if (detected) {
                telemetry.addLine("Tag of interest is in sight! ID: " + (parkingNum + 1));
            } else {
                telemetry.addLine("Could not find april tag! :(");
            }

            robot.update();
            telemetry.update();
        }

        robot.claw.close();

        robot.outtake.slides.slidesPercentMax = 1.0;

        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)), toCycle.end());
        drive.setPoseEstimate(origin);
        waitForStart();

        robot.claw.close();

        long startTime = System.currentTimeMillis();

        openCVWrapper.stop();

        robot.followTrajectorySequence(toPreload, this);

        while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT_AUTO) {
            robot.update();
        }

        robot.currentState = INTAKE_GLOBAL;
        robot.startIntakeGlobal(
                new Pose2d(cyclePose.getX() + 3, cyclePose.getY(), cyclePose.getHeading()),
                new Pose2d(69,12 * ySign),
                coneStackHeights[0]
        );

        robot.followTrajectorySequence(toCycle, this);

        robot.updateStayInPlacePID = true;

        for (int i = 0; i < cycles && (System.currentTimeMillis() - startTime <= timeToPark[parkingNum]); i++) {
            if (i != 0) {
                robot.currentState = INTAKE_GLOBAL;
                robot.startIntakeGlobal(
                        toCycle.end(),
                        new Pose2d(69,12 * ySign),
                        coneStackHeights[i]
                );
            }

            while ((robot.currentState == INTAKE_GLOBAL) && (System.currentTimeMillis() - startTime <= timeToPark[parkingNum])) {
                robot.update();
            }


            if (robot.sensors.robotNextToMe) {
                robot.startScoringGlobal(
                        new Pose2d(toCycle.end().getX(), toCycle.end().getY(), toCycle.end().getHeading()),
                        new Pose2d(21.0, 25.5 * ySign),
                        18.15);
            } else {
                robot.startScoringGlobal(
                        new Pose2d(toCycle.end().getX(), toCycle.end().getY(), toCycle.end().getHeading()),
                        new Pose2d(21.0, -2.5 * ySign),
                        27.25);
            }

            while ((robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT_AUTO) && (System.currentTimeMillis() - startTime <= timeToPark[parkingNum])) {
                robot.update();
            }
        }

        robot.currentState = INTAKE_RELATIVE;

        robot.update();
        while (!robot.outtake.isInPosition(5)) {
            robot.update();
        }

        robot.updateStayInPlacePID = false;
        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)), park[parkingNum].end());

        robot.followTrajectory(park[parkingNum], this);

        long clawStart = System.currentTimeMillis();
        robot.claw.park();
        robot.outtake.actuation.level();

        do  {
            robot.claw.park();
            robot.outtake.actuation.level();
            robot.update();
        } while (System.currentTimeMillis() - clawStart <= 2000);

        Storage.autoEndPose = drive.getPoseEstimate();
        Storage.isBlue = true;
    }
}