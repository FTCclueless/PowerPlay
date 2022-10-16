package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(group = "Test")
public class ServoTester extends LinearOpMode {
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
            robot.update();

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
            telemetry.update();
        }
    }
}
