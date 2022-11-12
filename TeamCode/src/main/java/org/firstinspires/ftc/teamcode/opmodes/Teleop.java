package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.claw.Claw;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.sensors.Sensors;

@TeleOp
public class Teleop extends LinearOpMode {

    boolean isBlue = true;
    double scoringHeight = 33.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        Claw claw = robot.claw;
        Sensors sensors = robot.sensors;

        robot.currentState = Robot.STATE.INTAKE_RELATIVE;

        waitForStart();

        while (!isStopRequested()) {
            // Driver A
            drive.drive(gamepad1);

            // get into intake position (just in case)
            if (gamepad1.a) {
                robot.startIntakeRelative();
            }

            if (robot.currentState == Robot.STATE.INTAKE_RELATIVE) {
                if (gamepad1.right_trigger > 0.5 || gamepad2.left_trigger > 0.5 ) {
                    claw.close();
                } else {
                    claw.open();
                }
            }

            if (gamepad1.b && robot.currentState == Robot.STATE.INTAKE_RELATIVE) {
                sensors.clawTouch = true;
            }

            // Driver B
            if ((robot.currentState == Robot.STATE.WAIT_FOR_START_SCORING && gamepad2.a) || robot.currentState == Robot.STATE.SCORING_RELATIVE) {
                robot.startScoringRelative(gamepad2, isBlue, scoringHeight);
            }

            if ((robot.currentState == Robot.STATE.SCORING_RELATIVE) && (gamepad2.right_trigger > 0.5)) {
                robot.startDepositing();
            }

            robot.update();
        }
    }
}
