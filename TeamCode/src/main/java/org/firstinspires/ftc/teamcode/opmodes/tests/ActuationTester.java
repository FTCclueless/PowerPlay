package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.actuation.Actuation;

@Disabled
@Config
@TeleOp(group = "Test")
public class ActuationTester extends LinearOpMode {

    public static double position = 0.0;
    long timeSinceLastActuationMovement = 0;
    boolean isTilt = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Actuation actuation = robot.outtake.actuation;
        robot.outtake.resetEncoders();
        robot.testMode();

        waitForStart();

        timeSinceLastActuationMovement = System.currentTimeMillis();

        while (!isStopRequested()) {
            if (System.currentTimeMillis() - timeSinceLastActuationMovement >= 1000) {
                if (isTilt) {
                    actuation.tilt();
                } else {
                    actuation.level();
                }
                isTilt = !isTilt;
                timeSinceLastActuationMovement = System.currentTimeMillis();
            }
            robot.update();
        }
    }
}
