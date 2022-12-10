package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.outtake.Outtake;

@Disabled
@Config
@TeleOp(group = "Test")
public class OuttakeTester extends LinearOpMode {
    public static double x = 5.0;
    public static double y = 0.0;
    public static double z = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.testMode();

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            robot.outtake.setTargetRelative(x,y,z);
        }
    }
}
