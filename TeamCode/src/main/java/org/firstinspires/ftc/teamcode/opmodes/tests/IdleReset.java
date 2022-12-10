package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Disabled
@Config
@Autonomous(group = "Reset")
public class IdleReset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.testMode();

        waitForStart();

        while(!isStopRequested()) {
           robot.currentState = Robot.STATE.INIT;
            robot.update();
        }
    }
}
