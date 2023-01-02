package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Robot.STATE.DEPOSIT;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_GLOBAL;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_RELATIVE;
import static org.firstinspires.ftc.teamcode.Robot.STATE.RETRACT;
import static org.firstinspires.ftc.teamcode.Robot.STATE.SCORING_GLOBAL;
import static org.firstinspires.ftc.teamcode.Robot.STATE.WAIT_FOR_START_SCORING;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.vision.OpenCVWrapper;

@Autonomous(group = "Test")
public class BlueTopAuto extends LinearOpMode {
    public static final int cycles = 5;
    public static int parkingNum = 1;
    public static final boolean lr = true; // Left : true | Right : false
    public static final boolean tb = true; // Top : true | Bottom : false

//    OpenCVWrapper openCVWrapper;

    double[] coneStackHeights = new double[]{4.15, 3.3, 2.25, 1.1, 0.0};
    ButtonToggle toggleA = new ButtonToggle();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;

//        openCVWrapper = new OpenCVWrapper(telemetry, hardwareMap, true);
//        assert(openCVWrapper != null);

        // Signs
        int xSign = tb ? 1 : -1;
        int ySign = lr ? 1 : -1;

        Pose2d origin = new Pose2d(
                36 * xSign,
                60 * ySign,
                lr ? Math.toRadians(90) : Math.toRadians(-90)
        );

        Pose2d toPose = new Pose2d(
                36,
                18,
                lr ? Math.toRadians(90) : Math.toRadians(-90)
        );

        Pose2d cyclePose = new Pose2d(
            47,
            12,
                tb ? Math.toRadians(180) : Math.toRadians(0)
        );

        TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
            .setReversed(true)
            .lineToConstantHeading(new Vector2d(toPose.getX(), toPose.getY()))
            .splineTo(new Vector2d(cyclePose.getX(), cyclePose.getY()), Math.toRadians(0))
            .addDisplacementMarker(40, () -> {
                robot.currentState = Robot.STATE.SCORING_GLOBAL;
                robot.startScoringGlobal(new Pose2d(toPose.getX(), toPose.getY(), toPose.getHeading()), new Pose2d(24 * xSign,ySign),28); // 36
            })
            .build();

        drive.setPoseEstimate(cyclePose);

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
                9,
                cyclePose.getY()
            )).build()
        };

        robot.resetEncoders();
        robot.claw.open();

//        openCVWrapper.init();
//        openCVWrapper.start();

        Log.e("camera setup", "");

        sleep(1000);
        robot.initPosition();

        Log.e("init position", "");

        while (opModeInInit()) {
            telemetry.setMsTransmissionInterval(50);

            boolean detected = false;

//            parkingNum = openCVWrapper.getParkingNum();
            detected = true;

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

        robot.drivetrain.setBreakFollowingThresholds(new Pose2d(0.5, 0.5, Math.toRadians(5)), park[parkingNum].end());
        drive.setPoseEstimate(origin);
        waitForStart();

//        openCVWrapper.stop();

        robot.followTrajectorySequence(to, this);

        while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT) {
            robot.update();
        }

        for (int i = 0; i < cycles; i++) {
            robot.currentState = INTAKE_GLOBAL;
            // TODO verify the x and y sign on this. It should not be like this
            robot.startIntakeGlobal(
                    to.end(),
                    new Pose2d(70 * xSign,12 * ySign),
                    coneStackHeights[i]
            );

            while (robot.currentState == INTAKE_GLOBAL) {
                robot.update();
            }

            robot.startScoringGlobal(new Pose2d(to.end().getX() + 2, to.end().getY(), to.end().getHeading()), new Pose2d(24 * xSign,0),28); // 36
            while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT) {
                robot.update();
            }
        }

        robot.currentState = INTAKE_RELATIVE;

        robot.update();
        while (!robot.outtake.isInPosition()) {
            robot.update();
        }
        robot.followTrajectory(park[parkingNum], this);

        long clawStart = System.currentTimeMillis();
        robot.claw.park();

        while (System.currentTimeMillis() - clawStart <= 300) {
            robot.claw.park();
            robot.claw.update();
        }

        Storage.autoEndPose = drive.getPoseEstimate();
        Storage.isBlue = true;

        Storage.autoEndPose = drive.getPoseEstimate();
        Storage.isBlue = true;
    }
}