package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Disabled
@Config
@TeleOp(group = "Test")
public class ActuationTester extends LinearOpMode {

    public static double position = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.outtake.resetEncoders();
        robot.testMode();

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            robot.actuation.setTargetActPosition(position);
        }
    }
}
