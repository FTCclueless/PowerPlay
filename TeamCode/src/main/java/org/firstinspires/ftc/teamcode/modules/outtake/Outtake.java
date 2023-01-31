package org.firstinspires.ftc.teamcode.modules.outtake;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.actuation.Actuation;
import org.firstinspires.ftc.teamcode.modules.extension.Extension;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.Model;
import org.firstinspires.ftc.teamcode.util.MotorPriority;
import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.Pose3D;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;
import java.util.Arrays;

public class Outtake {
    public Sensors sensors;

    public Turret turret;
    public Slides slides;
    public Extension extension;
    public Actuation actuation;

    ArrayList<MotorPriority> motorPriorities;

    double targetHeight = 0.0;
    double targetExtension = 0.0;

    public double targetTurretAngle = 0.0;
    public double targetSlidesLength = 0.0;
    public double targetExtensionLength = 0.0;

    double currentTurretAngle = 0.0;
    double currentSlidesLength = 0.0;
    double currentExtensionLength = 0.0;

    double turretXOffset = 0.0;
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
        actuation = new Actuation(hardwareMap, servos);
        extension = new Extension(hardwareMap, servos, this, actuation);
    }

    public void resetEncoders () {
        slides.resetEncoders();
        turret.resetEncoders();
    }

    public void updateTelemetry () {
        TelemetryUtil.packet.put("extensionIn", extensionIn + "");
    }

    boolean turretClips = false;
    boolean extensionIn = false;

    public void update() {
        updateRelativePos();

        turretClips = isTurretGoThroughBad();

        if (turretClips) {
            Log.e("TURRET CLIPS", "");
        }

        extensionIn = (currentExtensionLength <= (extension.minDistToNotHitMotor + 3));
        boolean backExtendCheck = targetSlidesLength <= 9 && (isTurretGoThroughRange(120, 185) || isTurretGoThroughRange(-185, -120));
        if(backExtendCheck) {
            extensionIn = (currentExtensionLength <= (extension.minDistToNotHitMotor + 5));
        }

        // if we are going to ram into drivetrain or extension is out and the turret is within 9
        if (!(turretClips && currentSlidesLength <= 9) && (extensionIn || turret.isInPosition(30, targetTurretAngle))) {
            turret.setTargetTurretAngle(targetTurretAngle);
        }

        if ((turretClips && targetSlidesLength <= 9 && extensionIn) || (targetSlidesLength <= (actuation.isLevel()?6:1.3) && currentExtensionLength < extension.minDistToNotHitMotor)) {
            Log.e("setting target slides length to 12", "");
            slides.setTargetSlidesLength(12);
        } else {
            if(extensionIn || slides.isInPosition(4, targetSlidesLength)) {
                slides.setTargetSlidesLength(targetSlidesLength);
            }
        }
        double targetExtent = 0;
        if (turret.isInPosition(15, targetTurretAngle) && slides.isInPosition(4, targetSlidesLength)) {
            targetExtent = targetExtensionLength;
        } else {
            targetExtent = extension.baseSlidesExtension;
            if (backExtendCheck) {
//                slides.setTargetSlidesLength(5);
                if (!Storage.isTeleop) {
                    targetExtent += 5.0;
                } else {
                    targetExtent += 1.25;
                }
//                if (extension.isInPosition(targetExtent,0.5)) {
//                    slides.setTargetSlidesLength(targetSlidesLength);
//                }
            }
        }

        Log.e("extension.currentExtensionLenght", extension.currentExtensionLength + "");

        if(!actuation.isLevel()) {
            targetExtent -= extension.actuationTiltDistance;
        }

        if ((targetExtent <= extension.minDistToNotHitMotor) && ((targetSlidesLength <= (actuation.isLevel()?6:1.3) && slides.currentSlidesLength <= 14) || (slides.currentSlidesLength < (actuation.isLevel()?6:1.3)))) {
            extension.retractExtension();
        } else {
            extension.setTargetExtensionLength(targetExtent);
        }

        Log.e("AFTERWARDS extension.targetExtensionLength", extension.targetExtensionLength + "");


//        else {
//            Log.e("extension.currentExtensionLength", extension.currentExtensionLength + "");
//            if (extension.currentExtensionLength >= 12) { // if the extension is far out, move extension first before slides
//                extension.setTargetExtensionLength(targetExtensionLength);
//                Log.e("extension.isInPosition", extension.isInPosition(1.5) + "");
//                if (extension.isInPosition(1.5)) {
//                    slides.setTargetSlidesLength(targetSlidesLength);
//                }
//            } else { // if extension is in, then move slides first
//                slides.setTargetSlidesLength(targetSlidesLength);
//            }
//        }
//        if (currentSlidesLength >= 9 || !turretClips) {
//            if (extension.currentExtensionLength >= 12) { // if extension is far out, move extension first before spinning turret
//                extension.setTargetExtensionLength(targetExtensionLength);
//                if (extension.isInPosition(1.5)) {
//                    turret.setTargetTurretAngle(targetTurretAngle);
//                }
//            } else { // if extension is in, then move turret first
//                turret.setTargetTurretAngle(targetTurretAngle);
//                if (turret.isInPosition(5) && slides.isInPosition(2)) { // once turret is in and slides are in correct position, move extension
//                    extension.setTargetExtensionLength(targetExtensionLength);
//                }
//            }
//        }

        slides.update();
        turret.update();
        actuation.update();
        extension.update();

        updateTelemetry();
    }

    public void retract()  {
        setTargetRelative(extension.baseSlidesExtension,0,0);
    }

    double a = Math.toRadians(17);
    double b = Math.toRadians(35);
    double c = Math.toRadians(-35);
    double d = Math.toRadians(-17);
    double e = Math.toRadians(95);
    double f = Math.toRadians(145);

    public boolean isTurretGoThroughBad() {
        double clipTarget = clipAngle(targetTurretAngle);
        double clipCurrent = clipAngle(currentTurretAngle);
        if (clipTarget == Math.min(Math.max(clipTarget,a),b)
                || clipCurrent == Math.min(Math.max(clipCurrent,a),b)
                || Math.signum(clipAngle(targetTurretAngle - (a+b)/2)) != Math.signum(clipAngle(currentTurretAngle  - (a+b)/2))){
            return true;
        }
        if (clipTarget == Math.min(Math.max(clipTarget,c),d)
                || clipCurrent == Math.min(Math.max(clipCurrent,c),d)
                || Math.signum(clipAngle(targetTurretAngle - (c+d)/2)) != Math.signum(clipAngle(currentTurretAngle  - (c+d)/2))){
            return true;
        }
        if (clipTarget == Math.min(Math.max(clipTarget,e),f)
                || clipCurrent == Math.min(Math.max(clipCurrent,e),f)
                || Math.signum(clipAngle(targetTurretAngle - (e+f)/2)) != Math.signum(clipAngle(currentTurretAngle  - (e+f)/2))){
            return true;
        }

        return false;
    }

    public boolean isTurretGoThroughRange(double a, double b) {
        a = Math.toRadians(a);
        b = Math.toRadians(b);
        double clipTarget = clipAngle(targetTurretAngle);
        double clipCurrent = clipAngle(currentTurretAngle);
        if (clipTarget == Math.min(Math.max(clipTarget,a),b)
                || clipCurrent == Math.min(Math.max(clipCurrent,a),b)
                || Math.signum(clipAngle(targetTurretAngle - (a+b)/2)) != Math.signum(clipAngle(currentTurretAngle  - (a+b)/2))){
            return true;
        }

        return false;
    }

    public void updateRelativePos() {
        currentTurretAngle = turret.getCurrentTurretAngle();
        currentSlidesLength = slides.getCurrentSlidesLength();
        currentExtensionLength = extension.getCurrentExtensionLength();
        if (!actuation.isLevel()) {
            currentExtensionLength += extension.actuationTiltDistance;
        }

        x = Math.cos(currentTurretAngle) * currentExtensionLength;
        y = Math.sin(currentTurretAngle) * currentExtensionLength;
        z = currentSlidesLength;
    }

    public void setTargetRelative(double targetX, double targetY, double targetZ) {
        targetHeight = Math.max(0, Math.min(targetZ, 39.08666));

        if (!actuation.isLevel()) {
            targetExtension = Math.min(extension.baseSlidesExtension + extension.strokeLength + extension.actuationTiltDistance-0.1, Math.max(extension.baseSlidesExtension, Math.sqrt(Math.pow((targetX),2) + Math.pow((targetY),2))));
        } else {
            targetExtension = Math.min(extension.baseSlidesExtension + extension.strokeLength, Math.max(extension.baseSlidesExtension, Math.sqrt(Math.pow((targetX),2) + Math.pow((targetY),2))));
        }

        targetTurretAngle = Math.atan2(targetY,targetX);
        targetExtensionLength = targetExtension;
        targetSlidesLength = Math.max(0, targetHeight);

        //Log.e("targetExtensionLength", targetExtensionLength + "");

//        if (isIntersectingRobot(targetX, targetY, targetZ)) { // checks if the target position is a valid position
//            targetExtensionLength = currentExtensionLength;
//            targetTurretAngle = currentTurretAngle;
//            targetSlidesLength = currentSlidesLength;
//
//            Log.e("INTERSECTION: ", "PLEASE BE AWARE!!");
//        }
    }

    public Pose2d findGlobalCoordinates (Pose2d robotPose, double xOffset, double yOffset) {
        double x = robotPose.getX() + xOffset*Math.cos(robotPose.getHeading()) - yOffset*Math.sin(robotPose.getHeading());
        double y = robotPose.getY() + yOffset*Math.cos(robotPose.getHeading()) + xOffset*Math.sin(robotPose.getHeading());

        return new Pose2d(x, y, robotPose.getHeading());
    }

    public void setTargetGlobal(Pose2d robotPose, Pose2d targetPose, double targetZ) {
        setTargetGlobal(robotPose, targetPose, targetZ, 0,0);
    }

    public void setTargetGlobal (Pose2d robotPose, Pose2d targetPose, double targetZ, double offsetX, double offsetY) {
        Pose2d turretPos = findGlobalCoordinates(robotPose, turretXOffset, turretYOffset);

        double deltaX = targetPose.getX() - turretPos.getX() + offsetX;
        double deltaY = targetPose.getY() - turretPos.getY() + offsetY;

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

    public Pose2d getGlobalArmPose (Pose2d robotPose) {
        return findGlobalCoordinates(robotPose, x,y);
    }

    public boolean isInPositionGlobal(Pose2d robotPose, Pose2d targetPose, double threshold) {
        Pose2d globalCoords = findGlobalCoordinates(robotPose, x,y);

        if ((Math.abs(globalCoords.getX() - targetPose.getX()) <= threshold) && (Math.abs(globalCoords.getY() - targetPose.getY()) <= threshold) && (slides.isInPosition(threshold))) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isInPosition() {
        return isInPosition(3);
    }

    public boolean isInPosition(double slidesThreshold) {
        return (turret.isInPosition(5, targetTurretAngle) && slides.isInPosition(slidesThreshold, targetSlidesLength) && extension.isInPosition(2));
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