package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@Autonomous(group = "Reset")
public class Reset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.testMode();

        waitForStart();

        while(!isStopRequested()) {
            robot.resetEncoders();
            robot.actuation.level();
            robot.outtake.extension.retractExtension();
            if (robot.outtake.extension.isInPosition(5)) {
                robot.claw.intake();
            }
            robot.update();
        }
    }
}
