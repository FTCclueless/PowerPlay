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
public class RedTeleop extends LinearOpMode {

    boolean isBlue = false;
    double scoringHeight = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        Claw claw = robot.claw;
        Actuation actuation = robot.actuation;
        Sensors sensors = robot.sensors;

        robot.currentState = Robot.STATE.INTAKE_RELATIVE;
        robot.isRelative = true;

        drive.localizer.setPoseEstimate(Storage.autoEndPose);

        ButtonToggle b_left_bumper = new ButtonToggle();

        while(opModeInInit()) {
            robot.update();
        }

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

            if ((robot.currentState == Robot.STATE.WAIT_FOR_START_SCORING) && (gamepad1.right_bumper)) {
                robot.outtake.slides.slidesPercentMax = 0.25;
                robot.currentState = Robot.STATE.INTAKE_RELATIVE;
            }

            if (gamepad2.a) { // low
                scoringHeight = 13;
            }

            if (gamepad2.b) { // medium
                scoringHeight = 21.5;
            }

            if (gamepad2.y) { // high (NEED TO CHANGE THE DEFAULT HEIGHT TOO)
                scoringHeight = 30;
            }

            if ((robot.currentState == Robot.STATE.WAIT_FOR_START_SCORING && (gamepad1.left_bumper || gamepad2.right_bumper)) || robot.currentState == Robot.STATE.SCORING_RELATIVE_WITH_IMU) {
                robot.startScoringRelative(gamepad2, isBlue, scoringHeight);
            }

            if (((robot.currentState == Robot.STATE.SCORING_RELATIVE_WITH_IMU) && (gamepad2.right_trigger > 0.5)) || ((robot.currentState == Robot.STATE.ADJUST) && (gamepad2.right_trigger > 0.5))) {
                robot.startDepositing();
            }

            if ((b_left_bumper.isClicked(gamepad2.left_bumper)) && (robot.currentState == Robot.STATE.SCORING_RELATIVE_WITH_IMU)) {
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