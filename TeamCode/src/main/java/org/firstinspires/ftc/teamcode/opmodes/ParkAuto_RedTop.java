package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.vision.OpenCVWrapper;

// HEY HEY ! DON'T READ THIS CODE! ITS FOR EMERGENCY
@Autonomous(group = "Test")
public class ParkAuto_RedTop extends LinearOpMode {
    public static int parkingNum = 0;
    public static final boolean lr = false; // Left : true | Right : false
    public static final boolean tb = true; // Top : true | Bottom : false

    OpenCVWrapper openCVWrapper;

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
                55 * xSign,
                14.5 * ySign,
                tb ? 0 : Math.PI
        );

        Pose2d depositPose = new Pose2d(
                38 * xSign,
                14.5 * ySign,
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

        // TODO clean this up a little? Kinda lookin a little bad
        Trajectory[] park = new Trajectory[] {
                drive.trajectoryBuilder(toDeposit.end()).strafeTo(new Vector2d(
                        origin.getX() + (24 * ySign),
                        depositPose.getY()
                )).build(),
                drive.trajectoryBuilder(toDeposit.end()).strafeTo(new Vector2d(
                        origin.getX() - (2.000001 * ySign),
                        depositPose.getY()
                )).build(),
                drive.trajectoryBuilder(toDeposit.end()).strafeTo(new Vector2d(
                        origin.getX() - (26 * ySign),
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

        robot.followTrajectorySequence(to, this);
        robot.followTrajectory(park[parkingNum], this);

        Storage.autoEndPose = drive.getPoseEstimate();
    }
}