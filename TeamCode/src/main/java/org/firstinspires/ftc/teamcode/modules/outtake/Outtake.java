package org.firstinspires.ftc.teamcode.modules.outtake;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.extension.Extension;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.Model;
import org.firstinspires.ftc.teamcode.util.MotorPriority;
import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;
import java.util.Arrays;

public class Outtake {
    public Sensors sensors;

    public Turret turret;
    public Slides slides;
    public Extension extension;

    ArrayList<MotorPriority> motorPriorities;

    double targetHeight = 0.0;
    double targetExtension = 0.0;

    public double targetTurretAngle = 0.0;
    public double targetSlidesLength = 0.0;
    public double targetExtensionLength = 0.0;

    double currentTurretAngle = 0.0;
    double currentSlidesLength = 0.0;
    double currentExtensionLength = 0.0;

    double turretXOffset = -2.0;
    double turretYOffset = 0.0;

    double x, y, z;

    Model leftDrivePod = new Model(Arrays.asList(new Pose3D(8.27, 7.965, 0), new Pose3D(-8.27, 5.6678, -4.21)));
    Model rightDrivePod = new Model(Arrays.asList(new Pose3D(8.27, -7.965, 0), new Pose3D(-8.27, -5.6678, -9)));
    Model center = new Model(Arrays.asList(new Pose3D(0, 5.6678, 0), new Pose3D(-8.27, -5.6678, -5.34)));

    ArrayList<MyServo> servos;

    public Outtake (HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors, ArrayList<MyServo> servos) {
        this.motorPriorities = motorPriorities;
        this.sensors = sensors;
        this.servos = servos;

        turret = new Turret(hardwareMap, motorPriorities, sensors, this);
        slides = new Slides(hardwareMap, motorPriorities, sensors, this);
        extension = new Extension(hardwareMap, servos, this);
    }

    public void resetEncoders () {
        slides.resetEncoders();
        turret.resetEncoders();
    }

    public void updateTelemetry () {
        TelemetryUtil.packet.put("isInPosition: ", isInPosition());
    }

    public void update() {
        updateRelativePos();

        boolean turretClips = isTurretGoThroughBad();

        if (turretClips && targetSlidesLength < 9){
            slides.setTargetSlidesLength(12);
        }
        else {
            slides.setTargetSlidesLength(targetSlidesLength);
        }
        if (currentSlidesLength >= 9 || !turretClips){
            turret.setTargetTurretAngle(targetTurretAngle);
            extension.setTargetExtensionLength(targetExtensionLength);
        }

        slides.update();
        turret.update();
        extension.update();

        updateTelemetry();
    }

    public void retract()  {
        extension.retractExtension();
        slides.setTargetSlidesLength(0);
        turret.setTargetTurretAngle(Math.toRadians(0));
    }

    double a = Math.toRadians(20);
    double b = Math.toRadians(35);
    double c = Math.toRadians(-35);
    double d = Math.toRadians(-20);
    double e = Math.toRadians(95);
    double f = Math.toRadians(145);

    public boolean isTurretGoThroughBad() {
        double clipTarget = clipAngle(targetTurretAngle);
        double clipCurrent = clipAngle(currentTurretAngle);
        if (clipTarget == Math.min(Math.max(clipTarget,a),b)
                || clipCurrent == Math.min(Math.max(clipCurrent,a),b)
                || Math.signum(targetTurretAngle - (a+b)/2) != Math.signum(currentTurretAngle  - (a+b)/2)){
            return true;
        }
        if (clipTarget == Math.min(Math.max(clipTarget,c),d)
                || clipCurrent == Math.min(Math.max(clipCurrent,c),d)
                || Math.signum(targetTurretAngle - (c+d)/2) != Math.signum(currentTurretAngle  - (c+d)/2)){
            return true;
        }

        return false;
    }

    public void updateRelativePos() {
        currentTurretAngle = turret.getCurrentTurretAngle();
        currentSlidesLength = slides.getCurrentSlidesLength();
        currentExtensionLength = extension.getCurrentExtensionLength();

        x = Math.cos(currentTurretAngle) * currentExtensionLength;
        y = Math.sin(currentTurretAngle) * currentExtensionLength;
        z = currentSlidesLength;
    }

    public void setTargetRelative(double targetX, double targetY, double targetZ) {
        targetHeight = Math.max(0, Math.min(targetZ, 39.08666));
        targetExtension = Math.min(extension.baseSlidesExtension + extension.strokeLength, Math.max(extension.baseSlidesExtension, Math.sqrt(Math.pow((targetX),2) + Math.pow((targetY),2))));

        targetTurretAngle = Math.atan2(targetY,targetX);
        targetExtensionLength = targetExtension;
        targetSlidesLength = Math.max(0, targetHeight);

        if (isIntersectingRobot(targetX, targetY, targetZ)) { // checks if the target position is a valid position
            targetExtensionLength = currentExtensionLength;
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
        return (turret.isInPosition(5) && slides.isInPosition(1.5) && extension.isInPosition(2));
    }

    public boolean isIntersectingRobot (double targetX, double targetY, double targetZ) {
        if (leftDrivePod.isIntersecting(targetX, targetY, targetZ)) {
            Log.e("************************INTERSECTING LEFT DRIVE POD************************", "");
            return true;
        } else if (center.isIntersecting(targetX, targetY, targetZ))  {
            Log.e("************************INTERSECTING CENTER************************", "");
            return true;
        } else if (rightDrivePod.isIntersecting(targetX, targetY, targetZ))  {
            Log.e("************************INTERSECTING RIGHT DRIVE POD************************", "");
            return true;
        }
        else {
            return false;
        }
    }
}