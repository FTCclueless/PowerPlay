package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.v4bar.V4Bar;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;

@TeleOp
@Config
public class SlidesPIDTuner extends LinearOpMode {

    int currentState = 1;

    public double error = 0.0;

    public static double p = 0.0;
    public static double i = 0.0;
    public static double d = 0.0;

    ButtonToggle a = new ButtonToggle();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Slides slides = robot.outtake.slides;
        V4Bar v4Bar = robot.outtake.v4Bar;

        robot.testMode();

        waitForStart();

        slides.setTargetSlidesLength(0.0);
        v4Bar.setTargetV4BarAngle(Math.toRadians(90));

        while (!isStopRequested()) {
            robot.update();

            error = slides.getCurrentSlidesLength() - slides.targetSlidesLength;
            slides.updateSlidesPID(p,i,d);

            if(a.isClicked(gamepad1.a)) {
                currentState++;
                if(currentState == 4) {
                    currentState = 1;
                }
            }

            switch(currentState) {
                case 1:
                    slides.setTargetSlidesLength(0);
                    break;
                case 2:
                    slides.setTargetSlidesLength(15);
                    break;
                case 3:
                    slides.setTargetSlidesLength(25);
                    break;
            }

            telemetry.addData("currentState: ", currentState);
            telemetry.addData("slides power: ", slides.slidesPower);
            telemetry.update();
        }
    }
}