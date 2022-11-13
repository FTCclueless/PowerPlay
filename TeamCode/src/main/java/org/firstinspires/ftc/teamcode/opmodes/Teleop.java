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
        robot.isRelative = true;

        waitForStart();

        while (!isStopRequested()) {
            // Driver A
            drive.drive(gamepad1);

            if (robot.currentState == Robot.STATE.INTAKE_RELATIVE) {
                if (gamepad1.right_trigger > 0.5 || gamepad2.left_trigger > 0.5 ) {
                    claw.close();
                } else {
                    claw.intake();
                }
            }

            if (gamepad1.a && robot.currentState == Robot.STATE.INTAKE_RELATIVE) {
                sensors.clawTouch = true;
            }

            // Driver B
            if ((robot.currentState == Robot.STATE.WAIT_FOR_START_SCORING && gamepad2.a) || robot.currentState == Robot.STATE.SCORING_RELATIVE) {
                robot.startScoringRelative(gamepad2, isBlue, scoringHeight);
            }

            if ((robot.currentState == Robot.STATE.SCORING_RELATIVE) && (gamepad2.right_trigger > 0.5) || (robot.currentState == Robot.STATE.ADJUST) && (gamepad2.right_trigger > 0.5)) {
                robot.startDepositing();
            }

            if ((robot.currentState == Robot.STATE.SCORING_RELATIVE) && (Math.abs(gamepad2.left_stick_x) > 0.5) // turret adjustment
                || (robot.currentState == Robot.STATE.SCORING_RELATIVE) && (Math.abs(gamepad2.left_stick_y) > 0.5) // slides up/down adjustment
                || (robot.currentState == Robot.STATE.SCORING_RELATIVE) && (Math.abs(gamepad2.right_stick_y) > 0.5) // v4bar + slides adjustment
                || (robot.currentState == Robot.STATE.ADJUST)
            ) {
                robot.startAdjusting(gamepad2);
            }

            robot.update();
        }
    }
}
