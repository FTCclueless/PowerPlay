package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;

@Config
@TeleOp(group = "Test")
public class ServoTester extends LinearOpMode {

    boolean manualMode = true;

    ButtonToggle toggleX = new ButtonToggle();

    public static double servoAngle = 0.0;
    public static int servoNumber = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        robot.testMode();

        double[] servoPos = new double[robot.servos.size()];
        for (int i = 0; i < robot.servos.size(); i ++){
            servoPos[i] = 0.5;
        }

        int servoIndex = 0;
        boolean lastX = false;
        boolean lastY = false;
        double numLoops = 0;
        double totalTime = 0;

        waitForStart();

        while (!isStopRequested()) {
            if (toggleX.isClicked(gamepad1.x)) {
                manualMode = !manualMode;
            }

            if (manualMode) {
                numLoops ++;
                if (gamepad1.a) {
                    servoPos[servoIndex] += 0.001;
                }
                if (gamepad1.b){
                    servoPos[servoIndex] -= 0.001;
                }
                servoPos[servoIndex] = Math.min(1.0,Math.max(0,servoPos[servoIndex]));

                long start = System.nanoTime();
                robot.servos.get(servoIndex).setPosition(servoPos[servoIndex],1.0);
                double elapsedTime = (System.nanoTime()-start)/1000000000.0;
                totalTime += elapsedTime;

                boolean x = gamepad1.x;
                if (x && !lastX){
                    servoIndex += robot.servos.size() - 1;
                }
                lastX = x;
                boolean y = gamepad1.y;
                if (y && !lastY){
                    servoIndex += 1;
                }
                lastY = y;
                servoIndex = servoIndex % robot.servos.size();

                telemetry.addData("servoNum", servoIndex);
                telemetry.addData("servoPos", servoPos[servoIndex]);
                telemetry.addData("averageServoTime", totalTime/numLoops);
            } else {
                robot.servos.get(servoNumber).setAngle(Math.toRadians(servoAngle));
                telemetry.addData("servoAngle", servoAngle);
                telemetry.addData("servoNumber", servoNumber);
            }
            telemetry.update();
        }
    }
}
