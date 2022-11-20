package org.firstinspires.ftc.teamcode.modules.drive.roadrunner.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Disabled
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.drive(gamepad1);
            robot.outtake.v4Bar.setTargetV4BarAngle(Math.toRadians(90));
            robot.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("leftEncoder Pos", robot.sensors.getLeftEncoderPos());
            telemetry.addData("rightEncoder Pos", robot.sensors.getRightEncoderPos());
            telemetry.addData("backEncoder Pos", robot.sensors.getBackEncoderPos());

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading (odo)", poseEstimate.getHeading());
            telemetry.addData("heading (imu)", drive.getRawExternalHeading());
            telemetry.update();
        }
    }
}
