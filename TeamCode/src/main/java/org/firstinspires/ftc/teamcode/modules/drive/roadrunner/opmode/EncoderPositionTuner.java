package org.firstinspires.ftc.teamcode.modules.drive.roadrunner.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.sensors.Sensors;

@TeleOp
@Disabled
public class EncoderPositionTuner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        Sensors sensors = robot.sensors;

        double ticksPerRotation = 8192.0;
        double wheelRadius = 0.6889764;
        double ticksToInches = (wheelRadius * Math.PI * 2.0) / ticksPerRotation;
        telemetry.addData("Assumption 1","the robot does move left/right or forward/backward");
        telemetry.addData("Assumption 2","the robot is only rotating");
        telemetry.update();
        waitForStart();

        double lastAngle = drive.getRawExternalHeading();
        double currentCumAngle = 0;
        while (opModeIsActive()){

            double turn = gamepad1.right_stick_x * 0.35;
            double p1 = turn;
            double p2 = turn;
            double p3 = -1 * turn;
            double p4 = -1 * turn;
            drive.setMotorPowers(p1, p2, p3, p4);

            double currentAngle = drive.getRawExternalHeading();
            double deltaAngle = currentAngle - lastAngle;
            while (deltaAngle >= Math.PI){
                deltaAngle -= Math.PI * 2;
            }
            while (deltaAngle <= -Math.PI){
                deltaAngle += Math.PI * 2;
            }
            currentCumAngle += deltaAngle;
            lastAngle = currentAngle;

            double left = sensors.getRightEncoderPos() * ticksToInches * sensors.getLeftEncoderScaleFactor();
            double right = sensors.getLeftEncoderPos() * ticksToInches *  sensors.getRightEncoderScaleFactor();
            double back = sensors.getBackEncoderPos() * ticksToInches *  sensors.getBackEncoderScaleFactor();

            double trackWidth = 0;
            if (currentCumAngle != 0) {
                trackWidth = (right - left) / currentCumAngle;
            }

            telemetry.addData("Cumulative Angle",Math.toDegrees(currentCumAngle));
            telemetry.addData("Track Width",trackWidth);

            telemetry.addData("Left Odo Y", (-1 * left / currentCumAngle));
            telemetry.addData("Right Odo Y",(-1 * right / currentCumAngle));
            telemetry.addData("Back Odo X",back/currentCumAngle);

            telemetry.addData("Left",left);
            telemetry.addData("Right",right);
            telemetry.addData("Back",back);
            telemetry.update();
        }
    }
}