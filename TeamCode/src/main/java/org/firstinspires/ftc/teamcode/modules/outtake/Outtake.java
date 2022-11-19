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
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;
import java.util.Arrays;

public class Outtake {
    public Sensors sensors;

    public Turret turret;
    public Slides slides;
    public V4Bar v4Bar;

    ArrayList<MotorPriority> motorPriorities;

    double v4BarLength = 12.0;

    double targetHeight = 0.0;
    double targetExtension = 0.0;

    public double targetTurretAngle = 0.0;
    public double targetSlidesLength = 0.0;
    public double targetV4BarAngle = 0.0;

    double currentExtension = 0.0;
    double currentHeight = 0.0;

    double currentTurretAngle = 0.0;
    double currentSlidesLength = 0.0;
    double currentV4BarAngle = 0.0;

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
        v4Bar = new V4Bar(hardwareMap, servos, this);
    }

    public void updateTelemetry () {
        TelemetryUtil.packet.put("targetHeight: ", targetHeight);
        TelemetryUtil.packet.put("targetExtension ", targetExtension);
    }

    long holdingTime = System.currentTimeMillis();

    public void update() {
        updateRelativePos();

        if ((targetSlidesLength + (Math.sin(targetV4BarAngle) * v4BarLength) <= 3) && (clipAngle(Math.abs(currentTurretAngle-targetTurretAngle)) > Math.toRadians(2.5)) && (System.currentTimeMillis() - holdingTime <= 750)) { // checks if the target height is low & turret isn't close to target turret angle
            slides.setTargetSlidesLength(4-(Math.signum(targetV4BarAngle)*v4BarLength)); // lifts slides up
            Log.e("avoiding hitting self", "");

            Log.e("targetSlidesLength", targetSlidesLength + "");
            Log.e("targetV4BarAngle", targetV4BarAngle + "");
            Log.e("Math.sin(targetV4BarAngle)", Math.sin(targetV4BarAngle) + "");
            Log.e("value", targetSlidesLength + Math.sin(targetV4BarAngle)*v4BarLength + "");
        } else { // only sets the v4bar and slides unless the turret is in position or the height is high
            holdingTime = System.currentTimeMillis();
            v4Bar.setTargetV4BarAngle(targetV4BarAngle);
            slides.setTargetSlidesLength(targetSlidesLength);
        }

        if (currentSlidesLength + Math.sin(currentV4BarAngle)*v4BarLength > 3) { // checks if the slides & v4bar are high
            turret.setTargetTurretAngle(targetTurretAngle);
        }


        slides.update();
        turret.update();
        v4Bar.update();

        updateTelemetry();
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
        targetHeight = Math.max(-10,Math.min(targetZ,32));
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

        if (isIntersectingRobot(targetX, targetY, targetZ)) { // checks if the target position is a valid position
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
        if(turret.isInPosition(5) && slides.isInPosition(2) && v4Bar.isInPosition(10)) {
            return true;
        } else {
            return false;
        }
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