package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;

        robot.currentState = Robot.STATE.RETRACT;

        waitForStart();

        while (!isStopRequested()) {
            robot.update();

            if(gamepad1.a) {
                robot.startClawIntake();
            }

            drive.drive(gamepad1);

            Pose2d poseEstimate = drive.getPoseEstimate();
        }
    }
}
