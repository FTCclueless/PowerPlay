package org.firstinspires.ftc.teamcode.modules.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.modules.v4bar.V4Bar;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.MotorPriority;

import java.util.ArrayList;

public class Outtake {
    Sensors sensors;

    Turret turret;
    Slides slides;
    V4Bar v4Bar;

    ArrayList<MotorPriority> motorPriorities;

    double v4BarLength = 5.0;

    double targetHeight = 0.0;
    double targetExtension = 0.0;

    double targetTurretAngle = 0.0;
    double targetSlidesLength = 0.0;
    double targetV4BarAngle = 0.0;

    double currentExtension = 0.0;
    double currentHeight = 0.0;

    double currentTurretAngle = 0.0;
    double currentSlidesLength = 0.0;
    double currentV4BarAngle = 0.0;

    double x, y, z;

    public Outtake (HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {
        this.motorPriorities = motorPriorities;
        this.sensors = sensors;

        slides = new Slides(hardwareMap, motorPriorities, sensors);
        turret = new Turret(hardwareMap, motorPriorities, sensors);
        v4Bar = new V4Bar(hardwareMap);
    }

    public void update() {
        updatePos();

        slides.setTargetSlidesLength(targetSlidesLength);
        turret.setTargetTurretAngle(targetTurretAngle);
        v4Bar.setTargetV4BarAngle(targetV4BarAngle);

        slides.update();
        turret.update();
        v4Bar.update();
    }

    public void updatePos() {
        currentTurretAngle = turret.getCurrentTurretAngle();
        currentSlidesLength = slides.getCurrentSlidesLength();
        currentV4BarAngle = v4Bar.getCurrentV4BarAngle();

        currentExtension = Math.cos(currentV4BarAngle) * v4BarLength;
        currentHeight = currentSlidesLength + (Math.sin(currentV4BarAngle) * v4BarLength);

        // targetSlidesLength = targetHeight - (Math.sin(currentV4BarAngle) * v4BarLength);

        x = Math.cos(currentTurretAngle) * currentExtension;
        y = Math.sin(currentTurretAngle) * currentExtension;
        z = currentHeight;
    }

    public void setTarget(double targetX, double targetY, double targetZ) {
        targetHeight = targetZ;
        targetExtension = Math.sqrt(Math.pow((targetX),2) + Math.pow((targetY),2));

        targetV4BarAngle = Math.acos(targetExtension / v4BarLength);
        targetSlidesLength = targetHeight - (Math.sin(targetV4BarAngle) * v4BarLength); // comment out this if you want the v4bar to stay horizontal as slides are moving and then uncomment line 69
        targetTurretAngle = Math.atan2(targetY,targetX);
    }
}