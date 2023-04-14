package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.actuation.Actuation;
import org.firstinspires.ftc.teamcode.modules.claw.Claw;
import org.firstinspires.ftc.teamcode.modules.claw.ConeFlipper;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.odoLifter.OdoLifter;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.Storage;

@TeleOp
public class Teleop extends LinearOpMode {

    boolean isBlue = true;
    double scoringHeight = 28;
    double initialAngle = Math.toRadians(180);
    double extraHeight = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        Claw claw = robot.claw;
        Actuation actuation = robot.outtake.actuation;
        Sensors sensors = robot.sensors;
        ConeFlipper coneFlipper = robot.coneFlipper;
        OdoLifter odoLifter = robot.odoLifter;

        drive.localizer.setPoseEstimate(Storage.autoEndPose);

        ButtonToggle a_x = new ButtonToggle();
        ButtonToggle b_dpad_up = new ButtonToggle();
        ButtonToggle b_left_trigger = new ButtonToggle();
        ButtonToggle a_left_dpad = new ButtonToggle();
        ButtonToggle a_right_dpad = new ButtonToggle();
        ButtonToggle a_b = new ButtonToggle();

        boolean extensionOut = false;

        Storage.isTeleop = true;

        robot.currentState = Robot.STATE.INTAKE_RELATIVE;
        robot.isRelative = true;
        robot.isTeleop = true;

        while(opModeInInit()) {
            coneFlipper.retract();
            odoLifter.up();
            robot.update();
        }

        waitForStart();

        while (!isStopRequested()) {
            // Driver A
            drive.drive(gamepad1);

            if (robot.currentState == Robot.STATE.INTAKE_RELATIVE) {
                if (gamepad1.right_trigger > 0.5) {
                    claw.close();
                    robot.alreadyClosed = true;
                } else {
                    if (robot.outtake.turret.targetTurretAngle == 0 && robot.outtake.turret.isInPosition(5)) {
                        claw.open();
                    } else {
                        claw.retractOpen();
                    }
                    robot.alreadyClosed = false;
                }
            }

            if (gamepad1.a && robot.currentState == Robot.STATE.INTAKE_RELATIVE) {
                sensors.coneInClaw = true;
            }

            if ((robot.currentState == Robot.STATE.WAIT_FOR_START_SCORING) && (gamepad1.y)) {
                extensionOut = false;
                robot.outtake.slides.slidesPercentMax = 1.0;
                robot.currentState = Robot.STATE.INTAKE_RELATIVE;
            }

//            if (a_x.isClicked(gamepad1.x) || b_dpad_up.isClicked(gamepad2.dpad_up)) {
//                robot.isWaitForStartScoring180 = !robot.isWaitForStartScoring180;
//            }

            if (gamepad2.dpad_up && robot.currentState == Robot.STATE.INTAKE_RELATIVE) {
                robot.intakeHeight += 0.3;
            }

            if (gamepad2.dpad_down && robot.currentState == Robot.STATE.INTAKE_RELATIVE) {
                robot.intakeHeight -= 0.3;
            }

            // con flippers
            if (a_left_dpad.isToggled(gamepad1.dpad_left)) {
                coneFlipper.downRight();
            } else {
                coneFlipper.upRight();
            }

            if (a_right_dpad.isToggled(gamepad1.dpad_right)) {
                coneFlipper.downLeft();
            } else {
                coneFlipper.upLeft();
            }

            // extendo intake
            if (robot.currentState == Robot.STATE.INTAKE_RELATIVE) {
                if(a_b.isClicked(gamepad1.b)) {
                    if (!extensionOut) {
                        robot.intakeExtensionDistance = 30;
                        extensionOut = true;
                    } else {
                        robot.intakeExtensionDistance = 0;
                        extensionOut = false;
                    }
                }
                if (gamepad1.dpad_up) {
                    robot.intakeExtensionDistance += 1.0;
                }
                if (gamepad1.dpad_down) {
                    robot.intakeExtensionDistance -= 1.0;
                }
            }

            // Driver B

            if (gamepad2.a) { // low
                scoringHeight = 8.3;
                robot.scoringLevel = 1;
            }

            if (gamepad2.b) { // medium
                scoringHeight = 17.5;
                robot.scoringLevel = 2;
            }

            if (gamepad2.y) { // high (NEED TO CHANGE THE DEFAULT HEIGHT TOO)
                scoringHeight = 26.0;
                robot.scoringLevel = 3;
            }

// checking for auto aim
//            if (gamepad1.left_bumper || gamepad2.left_bumper) {
//                robot.isAutoAim = true;
//            }
//
//            if (gamepad1.right_bumper || gamepad2.right_bumper) {
//                robot.isAutoAim = false;
//            }

            // if any buttons of the bumpers are clicked go to scoring relative
            if ((robot.currentState == Robot.STATE.WAIT_FOR_START_SCORING && (gamepad1.right_bumper || gamepad1.left_bumper || gamepad2.right_bumper || gamepad2.left_bumper)) || (robot.currentState == Robot.STATE.SCORING_RELATIVE)) {
                extensionOut = false;
                robot.startScoringRelative(gamepad2, isBlue, (scoringHeight));
            }

            // determines turret direction
            if ((gamepad1.right_bumper || gamepad2.right_bumper)) {
                robot.turnRightTurret = true;
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                robot.turnRightTurret = false;
            }

            if ((robot.currentState == Robot.STATE.SCORING_RELATIVE && (gamepad2.right_trigger > 0.5 || gamepad1.left_trigger > 0.5))) {
                Log.e("deposit", "");
                robot.startDepositing();
                initialAngle = Math.toRadians(180);
                extraHeight = 0.0;
                robot.tiltAct = true;
            }

            if ((b_left_trigger.isClicked(gamepad2.left_trigger > 0.5)) && (robot.currentState == Robot.STATE.SCORING_RELATIVE)) {
                if (actuation.isLevel()) {
                    Log.e("going to tilted", "");
                    actuation.tilt();
                } else {
                    Log.e("going to level", "");
                    actuation.level();
                }
            }

            robot.update();
        }
    }
}
