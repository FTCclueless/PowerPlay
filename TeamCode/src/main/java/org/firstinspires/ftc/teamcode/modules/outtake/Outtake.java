package org.firstinspires.ftc.teamcode.modules.outtake;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.modules.v4bar.V4Bar;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.Model;
import org.firstinspires.ftc.teamcode.util.MotorPriority;
import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.Pose3D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

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

    double turretXOffset = -2.0;
    double turretYOffset = 0.0;

    double x, y, z;

    Model leftDrivePod = new Model(Arrays.asList(new Pose3D(8.27, 7.965, 0), new Pose3D(-8.27, 5.6678, 4.21)));
    Model rightDrivePod = new Model(Arrays.asList(new Pose3D(8.27, -7.965, 0), new Pose3D(-8.27, -5.6678, 4.21)));
    Model center = new Model(Arrays.asList(new Pose3D(0, 5.6678, 0), new Pose3D(-8.27, -5.6678, 5.34)));

    ArrayList<MyServo> servos;

    public Outtake (HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors, ArrayList<MyServo> servos) {
        this.motorPriorities = motorPriorities;
        this.sensors = sensors;
        this.servos = servos;

        turret = new Turret(hardwareMap, motorPriorities, sensors);
        slides = new Slides(hardwareMap, motorPriorities, sensors);
        v4Bar = new V4Bar(hardwareMap, servos);
    }

    public void update() {
        updateRelativePos();

        slides.setTargetSlidesLength(targetSlidesLength);
        turret.setTargetTurretAngle(targetTurretAngle);
        v4Bar.setTargetV4BarAngle(targetV4BarAngle);

        slides.update();
        turret.update();
        v4Bar.update();
    }

    public void updateRelativePos() {
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

    public void setTargetRelative(double targetX, double targetY, double targetZ) {
        targetHeight = targetZ;
        targetExtension = Math.sqrt(Math.pow((targetX),2) + Math.pow((targetY),2));

        if (targetExtension > v4BarLength) {
            targetExtension = v4BarLength;
        }

        targetTurretAngle = Math.atan2(targetY,targetX);

        // this allows turret angle be more efficient by instead of doing full 180s move v4bar back
        if (Math.abs(targetTurretAngle) > Math.PI) { // Math.PI = 180 deg
            if(Math.abs(clipAngle(targetTurretAngle - Math.PI) - currentTurretAngle) < Math.abs(targetTurretAngle - currentTurretAngle)) { // this finds out which angle is closet to current and moves to that
                targetTurretAngle = clipAngle(targetTurretAngle - Math.PI);
                targetExtension *= -1;
            }
        }
        targetV4BarAngle = Math.acos(targetExtension / v4BarLength);
        targetSlidesLength = targetHeight - (Math.sin(targetV4BarAngle) * v4BarLength); // comment out this if you want the v4bar to stay horizontal as slides are moving and then uncomment line 69

        if (targetSlidesLength < 0) {
            targetV4BarAngle = (targetV4BarAngle * -1);

            if (targetV4BarAngle < Math.toRadians(-90)) { // in quadrant 3 --> will go into robot
                targetV4BarAngle += 2*Math.PI;
            }

            targetSlidesLength = targetHeight - (Math.sin(targetV4BarAngle) * v4BarLength);
        }

        if (isIntersectingRobot(targetX, targetY, targetZ)) {
            targetV4BarAngle = currentV4BarAngle;
            targetTurretAngle = currentTurretAngle;
            targetSlidesLength = currentSlidesLength;

            Log.e("INTERSECTION: ", "PLEASE BE AWARE!!");
        }
    }

    public Pose2d findGlobalCoordinates (Pose2d robotPose, double xOffset, double yOffset) {
        double x = robotPose.getX() + xOffset*Math.cos(robotPose.getHeading()) - yOffset*Math.sin(robotPose.getHeading());
        double y = robotPose.getY() + yOffset*Math.cos(robotPose.getHeading()) + xOffset*Math.sin(robotPose.getHeading());

        return new Pose2d(x, y, robotPose.getHeading());
    }

    public void setTargetGlobal (Pose2d robotPose, Pose2d targetPose, double targetZ) {
        Pose2d turretPos = findGlobalCoordinates(robotPose, turretXOffset, turretYOffset);

        double deltaX = targetPose.getX() - turretPos.getX();
        double deltaY = targetPose.getY() - turretPos.getY();

        double targetX = deltaX * Math.cos(robotPose.getHeading()) + deltaY * Math.sin(robotPose.getHeading());
        double targetY = deltaY * Math.cos(robotPose.getHeading()) - deltaX * Math.sin(robotPose.getHeading());

        setTargetRelative(targetX, targetY, targetZ);
    }

    public double clipAngle(double angle) {
        while (angle > Math.PI) {
            angle -= Math.PI * 2.0;
        }
        while (angle < -Math.PI) {
            angle += Math.PI * 2.0;
        }
        return angle;
    }

    public boolean isInPosition() {
        if(turret.isInPosition(5) && slides.isInPosition(5) && v4Bar.isInPosition(15)) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isIntersectingRobot (double targetX, double targetY, double targetZ) {
        if(leftDrivePod.isIntersecting(targetX, targetY, targetZ) || rightDrivePod.isIntersecting(targetX, targetY, targetZ) || center.isIntersecting(targetX, targetY, targetZ)) {
            return true;
        } else {
            return false;
        }
    }
}
