package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.modules.actuation.Actuation;
import org.firstinspires.ftc.teamcode.modules.extension.Extension;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.modules.v4bar.V4Bar;
import org.firstinspires.ftc.teamcode.util.ButtonToggle;

@TeleOp
@Config
public class TurretPIDTuner extends LinearOpMode {

    int currentState = 1;

    public double error = 0.0;

    public static double p = 3.2;
    public static double i = 0.0;
    public static double d = 0.0;

    ButtonToggle a = new ButtonToggle();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);
        Slides slides = robot.outtake.slides;
        Turret turret = robot.outtake.turret;
        Extension extension = robot.outtake.extension;
        Actuation actuation = robot.outtake.actuation;

        p = turret.turretPID.p;
        i = turret.turretPID.i;
        d = turret.turretPID.d;

        robot.testMode();
        robot.outtake.resetEncoders();

        waitForStart();

        slides.setTargetSlidesLength(8);
        turret.setTargetTurretAngle(Math.toRadians(0));
        actuation.level();
        extension.retractExtension();

        while (!isStopRequested()) {
            robot.update();

            error = turret.getCurrentTurretAngle() - turret.targetTurretAngle;
            turret.updateTurretPID(p,i,d);

            if(a.isClicked(gamepad1.a)) {
                currentState++;
                if(currentState == 4) {
                    currentState = 1;
                }
            }

            switch(currentState) {
                case 1:
                    turret.setTargetTurretAngle(Math.toRadians(0));
                    break;
                case 2:
                    turret.setTargetTurretAngle(Math.toRadians(90));
                    break;
                case 3:
                    turret.setTargetTurretAngle(Math.toRadians(180));
                    break;
            }

            telemetry.addData("currentState: ", currentState);
            telemetry.addData("turret power: ", turret.turretPower);
            telemetry.update();
        }
    }
}