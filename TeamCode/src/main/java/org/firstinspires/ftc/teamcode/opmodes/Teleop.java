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
    double scoringHeight = 26;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        Claw claw = robot.claw;
        Actuation actuation = robot.outtake.actuation;
        Sensors sensors = robot.sensors;

        drive.localizer.setPoseEstimate(Storage.autoEndPose);

        ButtonToggle a_x = new ButtonToggle();
        ButtonToggle b_dpad_up = new ButtonToggle();
        ButtonToggle b_left_trigger = new ButtonToggle();

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
                if (gamepad1.right_trigger > 0.5) {
                    claw.close();
                } else {
                    claw.open();
                }
            }

            if (gamepad1.a && robot.currentState == Robot.STATE.INTAKE_RELATIVE) {
                sensors.clawTouch = true;
            }

            if ((robot.currentState == Robot.STATE.WAIT_FOR_START_SCORING) && (gamepad1.b)) {
                robot.outtake.slides.slidesPercentMax = 0.1;
                robot.currentState = Robot.STATE.INTAKE_RELATIVE;
            }

            if (a_x.isClicked(gamepad1.x) || b_dpad_up.isClicked(gamepad2.dpad_up)) {
                robot.isWaitForStartScoring180 = !robot.isWaitForStartScoring180;
            }

            if (gamepad2.a) { // low
                scoringHeight = 8.3;
                robot.scoringLevel = 1;
            }

            if (gamepad2.b) { // medium
                scoringHeight = 17.5;
                robot.scoringLevel = 2;
            }

            if (gamepad2.y) { // high (NEED TO CHANGE THE DEFAULT HEIGHT TOO)
                scoringHeight = 26;
                robot.scoringLevel = 3;
            }

            // checking for auto aim
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                robot.isAutoAim = true;
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                robot.isAutoAim = false;
            }

            // if any buttons of the bumpers are clicked go to scoring relative
            if ((robot.currentState == Robot.STATE.WAIT_FOR_START_SCORING && (gamepad1.right_bumper || gamepad1.left_bumper || gamepad2.right_bumper || gamepad2.left_bumper)) || (robot.currentState == Robot.STATE.SCORING_RELATIVE)) {
                robot.startScoringRelative(gamepad2, Storage.isBlue, scoringHeight);
            }

            if ((robot.currentState == Robot.STATE.SCORING_RELATIVE && (gamepad2.right_trigger > 0.5 || gamepad1.left_trigger > 0.5))) {
                Log.e("deposit", "");
                robot.startDepositing();
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
