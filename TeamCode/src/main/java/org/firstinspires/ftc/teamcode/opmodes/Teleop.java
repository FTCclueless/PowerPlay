package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.actuation.Actuation;
import org.firstinspires.ftc.teamcode.modules.claw.Claw;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;
import org.firstinspires.ftc.teamcode.util.Storage;

@TeleOp
public class Teleop extends LinearOpMode {

    boolean isBlue = true;
    double scoringHeight = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        Claw claw = robot.claw;
        Actuation actuation = robot.actuation;
        Sensors sensors = robot.sensors;

        drive.localizer.setPoseEstimate(Storage.autoEndPose);

        ButtonToggle b_left_bumper = new ButtonToggle();

        while(opModeInInit()) {
            robot.outtake.setTargetRelative(3,0,0);
            robot.update();
        }

        robot.currentState = Robot.STATE.INTAKE_RELATIVE;
        robot.isRelative = true;
        robot.isTeleop = true;

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

            if ((robot.currentState == Robot.STATE.WAIT_FOR_START_SCORING) && (gamepad1.b)) {
                robot.currentState = Robot.STATE.INTAKE_RELATIVE;
            }

            if (gamepad2.a) { // low
                scoringHeight = 13;
                robot.scoringLevel = 1;
            }

            if (gamepad2.b) { // medium
                scoringHeight = 21.5;
                robot.scoringLevel = 2;
            }

            if (gamepad2.y) { // high (NEED TO CHANGE THE DEFAULT HEIGHT TOO)
                scoringHeight = 30;
                robot.scoringLevel = 3;
            }

            if (((robot.currentState == Robot.STATE.WAIT_FOR_START_SCORING || robot.currentState == Robot.STATE.SCORING_RELATIVE_AUTO_AIM) && (gamepad1.right_bumper || gamepad2.right_bumper)) || (robot.currentState == Robot.STATE.SCORING_RELATIVE)) {
                Log.e("startScoringRelative", "");
                robot.startScoringRelative(gamepad2, Storage.isBlue, scoringHeight);
            }

            if (((robot.currentState == Robot.STATE.WAIT_FOR_START_SCORING || robot.currentState == Robot.STATE.SCORING_RELATIVE) && (gamepad1.left_bumper || gamepad2.left_bumper)) || (robot.currentState == Robot.STATE.SCORING_RELATIVE_AUTO_AIM)) {
                Log.e("startScoringRelativeAutoAim", "");
                robot.startScoringRelativeAutoAim();
            }

            if (((robot.currentState == Robot.STATE.SCORING_RELATIVE || robot.currentState == Robot.STATE.SCORING_RELATIVE_AUTO_AIM) && (gamepad2.right_trigger > 0.5 || gamepad1.left_trigger > 0.5))) {
                Log.e("deposit", "");
                robot.startDepositing();
            }

            if ((b_left_bumper.isClicked(gamepad2.left_bumper)) && (robot.currentState == Robot.STATE.SCORING_RELATIVE)) {
                if (robot.actuation.isLevel()) {
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
