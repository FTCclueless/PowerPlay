package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.claw.Claw;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp
public class Teleop extends LinearOpMode {

    boolean isBlue = true;
    double scoringHeight = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        Claw claw = robot.claw;
        Sensors sensors = robot.sensors;

        robot.currentState = Robot.STATE.INTAKE_RELATIVE;
        robot.isRelative = true;

        drive.localizer.setPoseEstimate(PoseStorage.currentPose);

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

            if (gamepad1.b && robot.currentState == Robot.STATE.WAIT_FOR_START_SCORING) {
                robot.currentState = Robot.STATE.INTAKE_RELATIVE;
            }

            // Driver B
            if (gamepad2.x) { // ground
                scoringHeight = 2;
            }

            if (gamepad2.a) { // low
                scoringHeight = 10;
            }

            if (gamepad2.b) { // medium
                scoringHeight = 20;
            }

            if (gamepad2.y) { // high
                scoringHeight = 30;
            }

            if ((robot.currentState == Robot.STATE.WAIT_FOR_START_SCORING && gamepad2.right_bumper) || robot.currentState == Robot.STATE.SCORING_RELATIVE) {
                robot.startScoringRelative(gamepad2, isBlue, scoringHeight);
            }

            if (((robot.currentState == Robot.STATE.SCORING_RELATIVE) && (gamepad2.right_trigger > 0.5)) || ((robot.currentState == Robot.STATE.ADJUST) && (gamepad2.right_trigger > 0.5))) {
                robot.startDepositing();
            }

            robot.update();
        }
    }
}
