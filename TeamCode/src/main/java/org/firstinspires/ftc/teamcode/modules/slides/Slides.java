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


public class Slides {
    public DcMotorEx slide1, slide2;
    Sensors sensors;

    ArrayList<MotorPriority> motorPriorities;

    public PID slidesVelocityPID = new PID (0.0325,0.007,0.0);

    public double currentSlidesLength = 0.0;
    public double currentSlidesVelocity = 0.0;
    public double targetSlidesLength = 0.0;
    public double targetSlidesVelocity = 0.0;
    public double slidesPower = 0.0;
    public double slidesError = 0.0;
    public double slidesPercentMax = 0.5;

    // original: 82.9718558749
    public double maxSlidesSpeed = 82.9718558749; // inches per sec

    Outtake outtake;

    public Slides(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors, Outtake outtake) {
        this.motorPriorities = motorPriorities;
        this.sensors = sensors;
        this.outtake = outtake;

        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");

        slide2.setDirection(DcMotorSimple.Direction.REVERSE);

        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorPriorities.add(5, new MotorPriority(new DcMotorEx[] {slide1, slide2},3,4));
    }

    public void resetEncoders () {
        slide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("targetSlidesLength: ", targetSlidesLength);
        TelemetryUtil.packet.put("currentSlidesLength: ", currentSlidesLength);

        TelemetryUtil.packet.put("targetSlidesVelocity: ", targetSlidesVelocity);
        TelemetryUtil.packet.put("currentSlidesVelocity: ", currentSlidesVelocity);

        TelemetryUtil.packet.put("slidesPower: ", slidesPower);
        TelemetryUtil.packet.put("slidesError: ", slidesError);
    }

    boolean isVelocity = false;

    public void update() {
        updateSlidesValues();

        slidesError = targetSlidesLength - currentSlidesLength;

//        if (Math.abs(slidesError) <= 5) {
//            if (isVelocity) {
//                isVelocity = false;
//
//                slidesPID.p = slidesPositionalPID.p; // positional P value
//                slidesPID.i = slidesPositionalPID.i; // positional I value
//                slidesPID.d = slidesPositionalPID.d; // positional D value
//            }
//            slidesPower = slidesPID.update(slidesError);
//        } else {
//            if (!isVelocity) {
//                isVelocity = true;
//
//                slidesPID.p = slidesVelocityPID.p; // velocity P value
//                slidesPID.i = slidesVelocityPID.i; // velocity I value
//                slidesPID.d = slidesVelocityPID.d; // velocity D value
//            }
//            targetSlidesVelocity = Math.max(Math.min(slidesError * (maxSlidesSpeed/5), (maxSlidesSpeed*slidesPercentMax)),-maxSlidesSpeed*slidesPercentMax);
//            slidesPower = slidesPID.update(targetSlidesVelocity - currentSlidesVelocity);
//        }

        targetSlidesVelocity = Math.max(Math.min(slidesError * (maxSlidesSpeed/5), (maxSlidesSpeed*slidesPercentMax)),-maxSlidesSpeed*slidesPercentMax);
        slidesPower = slidesVelocityPID.update(targetSlidesVelocity - currentSlidesVelocity);
        motorPriorities.get(5).setTargetPower(-slidesPower);

        updateTelemetry();
    }

    public void updateSlidesValues() {
        currentSlidesLength = sensors.getSlidesLength();
        currentSlidesVelocity = sensors.getSlidesVelocity();
    }

    public void setTargetSlidesLength (double amount) {
        targetSlidesLength = amount;
        outtake.targetSlidesLength = amount;
    }

    public double getCurrentSlidesLength () {
        return currentSlidesLength;
    }

    public boolean isInPosition (double inches) {
        return (Math.abs(targetSlidesLength - currentSlidesLength) <= inches);
    }

    public void updateSlidesPID (double p, double i, double d) {
        slidesVelocityPID.p = p;
        slidesVelocityPID.i = i;
        slidesVelocityPID.d = d;
    }
}
