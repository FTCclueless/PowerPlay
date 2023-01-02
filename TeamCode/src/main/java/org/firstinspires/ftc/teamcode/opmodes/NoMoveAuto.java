package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.Robot.STATE.DEPOSIT;
import static org.firstinspires.ftc.teamcode.Robot.STATE.INTAKE_GLOBAL;
import static org.firstinspires.ftc.teamcode.Robot.STATE.SCORING_GLOBAL;
import static org.firstinspires.ftc.teamcode.Robot.STATE.WAIT_FOR_START_SCORING;

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

@Disabled
@Autonomous(group = "Test")
public class NoMoveAuto extends LinearOpMode {
    public static final int cycles = 5;
    public static int parkingNum = 0;
    public static final boolean lr = true; // Left : true | Right : false
    public static final boolean tb = true; // Top : true | Bottom : false
    //public static final double cycleBack = 6; // Once robot gets to cycle position how much it moves backwards
    //public static final double cycleY = 48; // Turning can give an offset (+ cone location)

//    OpenCVWrapper openCVWrapper;

    double[] coneStackHeights = new double[]{4.15, 3.3, 2.4, 1.435, 0.0};
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
            62.25 * ySign,
            Math.PI / 2 * (!lr ? -1 : 1)
        );

        Pose2d cyclePose = new Pose2d(
            45.75,
            10.75,
            0
        );

        //TrajectorySequence to = drive.trajectorySequenceBuilder(origin)
        //        .strafeTo()
        //    .build();

        drive.setPoseEstimate(cyclePose);

        // TODO clean this up a little? Kinda lookin a little bad
        Trajectory[] park = new Trajectory[]{
            drive.trajectoryBuilder(cyclePose).strafeTo(new Vector2d(
                origin.getX() + (23.5 + (tb ? 0 : 1.5)),
                cyclePose.getY()
            )).build(),
            drive.trajectoryBuilder(cyclePose).strafeTo(new Vector2d(
                origin.getX() - (2.000001 * ySign),
                cyclePose.getY()
            )).build(),
            drive.trajectoryBuilder(cyclePose).strafeTo(new Vector2d(
                origin.getX() - (27),
                cyclePose.getY()
            )).build()
        };

        robot.resetEncoders();
        robot.claw.open();

//        openCVWrapper.init();
//        openCVWrapper.start();

//        while (opModeInInit()) {
//            telemetry.setMsTransmissionInterval(50);
//
//            boolean detected = false;
//
//            /////
//            parkingNum = openCVWrapper.getParkingNum();
//            detected = true;  //should we always set to true ??? It is only used to send telemetry anyways
//            ///////////
//
//            robot.outtake.actuation.level();
//            robot.outtake.extension.retractExtension();
//            if (toggleA.isClicked(gamepad1.a)) {
//                robot.claw.close();
//            }
//
//            robot.update();
//
//            if (detected) {
//                telemetry.addLine(String.format("Tag of interest is in sight! ID: %d", parkingNum + 1));
//            } else {
//                telemetry.addLine("Could not find april tag! :(");
//            }
//
//            telemetry.update();
//        }
//
        waitForStart();
//
//        openCVWrapper.stop();

        // preload

        robot.currentState = SCORING_GLOBAL;

        robot.startScoringGlobal(cyclePose, new Pose2d(24 * xSign,0 * ySign),27, xSign * ySign); // 36
        while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT) {
            robot.update();
        }

        while (robot.outtake.slides.currentSlidesLength > 15) {
            robot.update();
        }

        // cycles

        for (int i = 0; i < cycles; i++ ) {
            robot.currentState = Robot.STATE.RETRACT;

            robot.startIntakeGlobal(
                cyclePose,
                new Pose2d(70 * xSign, 12 * ySign),
                coneStackHeights[i]
            );

            robot.update();
            while (robot.currentState == INTAKE_GLOBAL || robot.currentState == WAIT_FOR_START_SCORING) {
                robot.update();
            }

            robot.startScoringGlobal(
                cyclePose,
                new Pose2d(24 * xSign, 0),
                27,
                xSign * ySign
            );

            robot.update();
            while (robot.currentState == SCORING_GLOBAL || robot.currentState == DEPOSIT) {
                robot.update();
            }
        }

        Storage.autoEndPose = drive.getPoseEstimate();
        Storage.isBlue = true;
    }
}