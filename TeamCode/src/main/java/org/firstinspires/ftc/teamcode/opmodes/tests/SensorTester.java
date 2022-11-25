package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
@TeleOp(group = "drive")
public class SensorTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.resetEncoders();

        waitForStart();

        while (!isStopRequested()) {
            robot.sensors.updateHub1();
            robot.sensors.updateHub2();

            telemetry.addData("leftEncoder Pos", robot.sensors.getLeftEncoderPos());
            telemetry.addData("rightEncoder Pos", robot.sensors.getRightEncoderPos());
            telemetry.addData("backEncoder Pos", robot.sensors.getBackEncoderPos());
            telemetry.addData("current slides length", robot.sensors.slidesLength);
            telemetry.addData("current turret angle", Math.toDegrees(robot.sensors.turretAngle));

            telemetry.addData("clawLimit", robot.sensors.clawTouched());
            telemetry.update();
        }
    }
}
