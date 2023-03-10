package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(group = "Test")
public class ExtensionTester extends LinearOpMode {

    public static double length = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.outtake.resetEncoders();
        robot.testMode();

        waitForStart();

        robot.poleAlignment.oversideRetract();
        while (!isStopRequested()) {
            robot.outtake.actuation.level();
            robot.outtake.extension.setTargetExtensionLength(robot.outtake.extension.baseSlidesExtension + length);
            robot.outtake.slides.setTargetSlidesLength(10);
            robot.update();
        }
    }
}
