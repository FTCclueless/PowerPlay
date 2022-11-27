package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;

@Autonomous
public class MaxSpeedTester extends LinearOpMode {
    @Override
public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            robot.drivetrain.setMotorPowers(1.0,1.0,1.0,1.0);
            robot.drivetrain.update();

            robot.drivetrain.leftFront.setPower(1.0);
            robot.drivetrain.leftRear.setPower(1.0);
            robot.drivetrain.rightFront.setPower(1.0);
            robot.drivetrain.rightRear.setPower(1.0);
        }
    }
}
