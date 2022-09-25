package org.firstinspires.ftc.teamcode.modules.slides;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.MotorPriority;
import org.firstinspires.ftc.teamcode.util.PID;

import java.util.HashMap;


public class Slides {
    public DcMotorEx slides;
    Sensors sensors;

    HashMap<String, MotorPriority> motorPriorities;
    public enum STATE {PICKUP, GROUND, LOW, MEDIUM, HIGH}

    public STATE currentState = STATE.PICKUP;
    public PID slidesPID = new PID(0.0225,0.008,0.000007);

    public double currentSlidesLength = 0.0;
    public double currentSlidesVelocity = 0.0;
    public double targetSlidesLength = 0.0;
    public double targetSlidesVelocity = 0.0;
    public double slidesPower = 0.0;
    public static double slidesPercentMax = 0.98;

    public Slides(HardwareMap hardwareMap, HashMap<String, MotorPriority> motorPriorities, Sensors sensors) {
        this.motorPriorities = motorPriorities;
        this.sensors = sensors;

        // TODO: Implement 2 slides motor
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        motorPriorities.put("slides", new MotorPriority(slides,3,4));
    }

    public void update() {
        updateSlidesValues();

        double slidesError = targetSlidesLength - currentSlidesLength;
        targetSlidesVelocity = Math.max(Math.min(slidesError * (47.141223206685275/2), (47.141223206685275*slidesPercentMax)),-47.141223206685275*slidesPercentMax);
        slidesPower = slidesPID.update(targetSlidesVelocity - currentSlidesVelocity);
        motorPriorities.get("slides").setTargetPower(slidesPower);

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
}
