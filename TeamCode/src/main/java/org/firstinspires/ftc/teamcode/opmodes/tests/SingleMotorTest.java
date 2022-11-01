package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;

@TeleOp
public class SingleMotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Drivetrain drive = robot.drivetrain;
        ButtonToggle toggleX = new ButtonToggle();

        int state = 0;
        double speed = 0.25;

        waitForStart();

        while (!isStopRequested()) {
            if (toggleX.isClicked(gamepad1.x)) {
                if(state == 4) {
                    state = 0;
                } else {
                    state++;
                }
            }

            switch (state) {
                case 0:
                    drive.setMotorPowers(speed,0,0,0);
                    telemetry.addData("motor: ", "leftFront");
                    break;
                case 1:
                    drive.setMotorPowers(0,speed,0,-0);
                    telemetry.addData("motor: ", "leftRear");
                    break;
                case 2:
                    drive.setMotorPowers(0,0,speed,0);
                    telemetry.addData("motor: ", "rightRear");
                    break;
                case 3:
                    drive.setMotorPowers(0,0,0, speed);
                    telemetry.addData("motor: ", "rightFront");
                    break;
            }

            telemetry.update();
            robot.update();
        }
    }
}
