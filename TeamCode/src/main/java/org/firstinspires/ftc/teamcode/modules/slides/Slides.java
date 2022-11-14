package org.firstinspires.ftc.teamcode.modules.slides;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.outtake.Outtake;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.MotorPriority;
import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;


public class Slides {
    public DcMotorEx slide1, slide2;
    Sensors sensors;

    ArrayList<MotorPriority> motorPriorities;

//    public PID slidesPID = new PID(0.0215,0.01,0.0001);
    public PID slidesPID = new PID(0.017,0.007,0.0);

    public double currentSlidesLength = 0.0;
    public double currentSlidesVelocity = 0.0;
    public double targetSlidesLength = 0.0;
    public double targetSlidesVelocity = 0.0;
    public double slidesPower = 0.0;
    public double slidesError = 0.0;
    public static double slidesPercentMax = 0.98;

    double maxSlidesSpeed = 82.9718558749; // inches per sec

    Outtake outtake;

    public Slides(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors, Outtake outtake) {
        this.motorPriorities = motorPriorities;
        this.sensors = sensors;
        this.outtake = outtake;

        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

        slide1.setDirection(DcMotorSimple.Direction.REVERSE);

        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorPriorities.add(5, new MotorPriority(new DcMotorEx[] {slide1, slide2},3,4));
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("targetSlidesLength: ", targetSlidesLength);
        TelemetryUtil.packet.put("currentSlidesLength: ", currentSlidesLength);

//        TelemetryUtil.packet.put("targetSlidesVelocity: ", targetSlidesVelocity);
//        TelemetryUtil.packet.put("currentSlidesVelocity: ", currentSlidesVelocity);
//
//        TelemetryUtil.packet.put("slidesPower: ", slidesPower);
//        TelemetryUtil.packet.put("slidesError: ", slidesError);
    }

    public void update() {
        updateSlidesValues();

        slidesError = targetSlidesLength - currentSlidesLength;

        if (slidesError <= 5) {
            slidesPID.p = // slow P value
            slidesPID.i = // slow I value
            slidesPID.d = // slow D value
        } else {
            slidesPID.p = // fast P value
            slidesPID.i = // fast I value
            slidesPID.d = // fast D value
        }

        targetSlidesVelocity = Math.max(Math.min(slidesError * (maxSlidesSpeed/2), (maxSlidesSpeed*slidesPercentMax)),-maxSlidesSpeed*slidesPercentMax);
        slidesPower = slidesPID.update(targetSlidesVelocity - currentSlidesVelocity);
        motorPriorities.get(5).setTargetPower(slidesPower);

        updateTelemetry();
    }

    public void updateSlidesValues() {
        currentSlidesLength = sensors.getSlidesLength();
        currentSlidesVelocity = sensors.getSlidesVelocity();
    }

    public void moveToPickup() {
        setTargetSlidesLength(2.5);
    }

    public void moveToGround() {
        setTargetSlidesLength(5.0);
    }

    public void moveToLow() {
        setTargetSlidesLength(10.0);
    }

    public void moveToMedium() {
        setTargetSlidesLength(20.0);
    }

    public void moveToHigh() {
        setTargetSlidesLength(40.0);
    }

    public void moveUp (double amount) {
        setTargetSlidesLength(targetSlidesLength += amount);
    }

    public void moveDown (double amount) {
        setTargetSlidesLength(targetSlidesLength -= amount);
    }

    public void setTargetSlidesLength (double amount) {
        targetSlidesLength = amount;
        outtake.targetSlidesLength = amount;
    }

    public double getCurrentSlidesLength () {
        return currentSlidesLength;
    }

    public boolean isInPosition (double inches) {
        if (Math.abs(targetSlidesLength - currentSlidesLength) <= inches) {
            return true;
        } else {
            return false;
        }
    }

    public void updateSlidesPID (double p, double i, double d) {
        slidesPID.p = p;
        slidesPID.i = i;
        slidesPID.d = d;
    }
}
