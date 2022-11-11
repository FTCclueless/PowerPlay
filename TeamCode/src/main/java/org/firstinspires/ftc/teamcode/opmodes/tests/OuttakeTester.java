package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(group = "Test")
public class OuttakeTester extends LinearOpMode {

    public static double servoAngle = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.testMode();

        waitForStart();

        while (!isStopRequested()) {
            robot.servos.get(0).setAngle(Math.toRadians(servoAngle));
            robot.servos.get(1).setAngle(-Math.toRadians(servoAngle));

            telemetry.addData("servoAngle", servoAngle);
            telemetry.update();
        }
    }
}
