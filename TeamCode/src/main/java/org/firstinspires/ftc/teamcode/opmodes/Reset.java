package org.firstinspires.ftc.teamcode.opmodes;

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
        robot.currentState = Robot.STATE.IDLE;
        robot.resetEncoders();
        robot.outtake.slides.setTargetSlidesLength(0);
        robot.update();

        waitForStart();

        while(!isStopRequested()) {
            robot.outtake.actuation.level();
            robot.outtake.extension.retractExtension();
            robot.outtake.slides.setTargetSlidesLength(0);
            if (robot.outtake.extension.isInPosition(5)) {
                robot.claw.open();
            }
            robot.update();
        }
    }
}
