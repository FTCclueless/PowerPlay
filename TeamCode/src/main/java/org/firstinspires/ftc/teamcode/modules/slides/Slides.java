package org.firstinspires.ftc.teamcode.modules.slides;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.MotorPriority;
import org.firstinspires.ftc.teamcode.util.PID;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;


public class Slides {
    public DcMotorEx slide1, slide2;
    Sensors sensors;

    ArrayList<MotorPriority> motorPriorities;

    public PID slidesPID = new PID(0.0225,0.008,0.000007);

    public double currentSlidesLength = 0.0;
    public double currentSlidesVelocity = 0.0;
    public double targetSlidesLength = 0.0;
    public double targetSlidesVelocity = 0.0;
    public double slidesPower = 0.0;
    public static double slidesPercentMax = 0.98;

    public Slides(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {
        this.motorPriorities = motorPriorities;
        this.sensors = sensors;

        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        motorPriorities.add(5, new MotorPriority(new DcMotorEx[] {slide1, slide2},3,4));
    }

    public void update() {
        updateSlidesValues();

        double slidesError = targetSlidesLength - currentSlidesLength;
        targetSlidesVelocity = Math.max(Math.min(slidesError * (47.141223206685275/2), (47.141223206685275*slidesPercentMax)),-47.141223206685275*slidesPercentMax);
        slidesPower = slidesPID.update(targetSlidesVelocity - currentSlidesVelocity);
        motorPriorities.get(5).setTargetPower(slidesPower);
    }

    public void updateSlidesValues() {
        currentSlidesLength = sensors.getSlidesLength();
        currentSlidesVelocity = sensors.getSlidesVelocity();
    }

    public void moveToPickup() {
        targetSlidesLength = 2.5;
    }

    public void moveToGround() {
        targetSlidesLength = 5.0;
    }

    public void moveToLow() {
        targetSlidesLength = 10.0;
    }

    public void moveToMedium() {
        targetSlidesLength = 20.0;
    }

    public void moveToHigh() {
        targetSlidesLength = 40.0;
    }

    public void moveUp (double amount) {
        targetSlidesLength += amount;
    }

    public void moveDown (double amount) {
        targetSlidesLength -= amount;
    }

    public void setTargetSlidesLength (double amount) {
        targetSlidesLength = amount;
    }

    public double getCurrentSlidesLength () {
        return currentSlidesLength;
    }

    public boolean isInPosition (double inches) {
        if(Math.abs(targetSlidesLength - currentSlidesLength) <= inches) {
            return true;
        } else {
            return false;
        }
    }
}
