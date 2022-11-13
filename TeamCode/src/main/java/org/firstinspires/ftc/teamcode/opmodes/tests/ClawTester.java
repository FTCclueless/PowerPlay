package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.claw.Claw;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

@Config
@TeleOp(group = "Test")
public class ClawTester extends LinearOpMode {
    int state = 0;

    public static double openPosition = 0.0;
    public static double intakePosition = 0.0;
    public static double closePosition = 0.0; //0.096

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Claw claw = robot.claw;
        robot.testMode();

        intakePosition = claw.intakePosition;
        openPosition = claw.openPosition;
        closePosition = claw.closePosition;

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            if (gamepad1.a) {
                state++;
                if (state > 2) {
                    state = 0;
                }
            }

            switch (state) {
                case 0:
                    claw.setTargetClawPosition(intakePosition);
                    break;
                case 1:
                    claw.setTargetClawPosition(openPosition);
                    break;
                case 2:
                    claw.setTargetClawPosition(closePosition);
                    break;
            }
        }
    }
}
