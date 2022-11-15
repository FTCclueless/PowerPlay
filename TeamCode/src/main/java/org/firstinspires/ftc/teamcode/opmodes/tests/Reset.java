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
            robot.outtake.v4Bar.setTargetV4BarAngle(Math.toRadians(90));
            if (robot.outtake.v4Bar.isInPosition(5)) {
                robot.claw.close();
            }
            robot.update();
        }
    }
}
