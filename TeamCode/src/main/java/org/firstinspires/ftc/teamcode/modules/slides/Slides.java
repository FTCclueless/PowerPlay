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
    public enum STATE {PICKUP, GROUND, LOW, MEDIUM, HIGH, ADJUST}

    public STATE currentState = STATE.PICKUP;
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

        switch (currentState) {
            case PICKUP:
                targetSlidesLength = 2.5;
                break;
            case GROUND:
                targetSlidesLength = 5.0;
                break;
            case LOW:
                targetSlidesLength = 10.0;
                break;
            case MEDIUM:
                targetSlidesLength = 20.0;
                break;
            case HIGH:
                targetSlidesLength = 40.0;
                break;
            case ADJUST:
                break;
        }
    }

    public void updateSlidesValues() {
        currentSlidesLength = sensors.getSlidesLength();
        currentSlidesVelocity = sensors.getSlidesVelocity();
    }

    public void moveToPickup() {
        currentState = STATE.PICKUP;
    }

    public void moveToGround() {
        currentState = STATE.GROUND;
    }

    public void moveToLow() {
        currentState = STATE.LOW;
    }

    public void moveToMedium() {
        currentState = STATE.MEDIUM;
    }

    public void moveToHigh() {
        currentState = STATE.HIGH;
    }

    public void moveUp (double amount) {
        currentState = STATE.ADJUST;
        targetSlidesLength += amount;
    }

    public void moveDown (double amount) {
        currentState = STATE.ADJUST;
        targetSlidesLength -= amount;
    }
}
