package org.firstinspires.ftc.teamcode.sensors;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.modules.drive.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.MotorPriority;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;
import java.util.List;

public class Sensors {
    ArrayList<MotorPriority> motorPriorities;
    HardwareMap hardwareMap;
    ThreeWheelLocalizer localizer;

    public double slidesLength, slidesVelocity, slides1Current, slides2Current;
    public double turretAngle, turretVelocity;

    private final VoltageSensor batteryVoltageSensor;
    private final AnalogInput leftUltrasonic;
//    private final ColorSensor clawColor;

    public double leftDist = 0.0;
    public double clawColorReading = 0.0;
    public boolean coneInClaw = false;
    public boolean robotNextToMe = false;

    public Sensors (HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, ThreeWheelLocalizer localizer) {
        this.motorPriorities = motorPriorities;
        this.hardwareMap = hardwareMap;
        this.localizer = localizer;

        // setup bulkReads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        leftUltrasonic = hardwareMap.get(AnalogInput.class, "leftUltrasonic");
//        clawColor = hardwareMap.get(ColorSensor.class, "clawColor");
    }

    public void updateTelemetry () {
        TelemetryUtil.packet.put("slides1Current: ", slides1Current);
        TelemetryUtil.packet.put("slides2Current: ", slides2Current);
        TelemetryUtil.packet.put("leftDist: ", leftDist);

        Log.e("robotNextToMeCounter", robotNextToMeCounter + "");
    }

    int robotNextToMeCounter;

    public void updateHub1() {
        try {
            localizer.encoders[0].update(motorPriorities.get(0).motor[0].getCurrentPosition()); // left
            localizer.encoders[1].update(motorPriorities.get(3).motor[0].getCurrentPosition()); // right
            localizer.encoders[2].update(motorPriorities.get(1).motor[0].getCurrentPosition()); // back

            leftDist = leftUltrasonic.getVoltage();

            if (leftDist < 0.1) {
                robotNextToMeCounter += 1;
            } else {
                robotNextToMeCounter -= 1;
            }

            robotNextToMeCounter = Math.max(0, Math.min(robotNextToMeCounter, 10));

            robotNextToMe = robotNextToMeCounter > 5;

//            clawColorReading = clawColor.argb() / 10000000;
//            clawColorReading = clawColor.alpha();
        }
        catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "control hub failed");
        }
    }

    double turretTicksToRadian = 336.607748257;
    double slidesTickToInch = 33.5162937069;

    public void updateHub2() {
        try {
            turretAngle = motorPriorities.get(4).motor[0].getCurrentPosition() / turretTicksToRadian * -1; // radians of turret
            turretVelocity = motorPriorities.get(4).motor[0].getVelocity() / turretTicksToRadian * -1;

            slidesLength = motorPriorities.get(5).motor[1].getCurrentPosition() / slidesTickToInch * -1; // inches of slides
            slidesVelocity = motorPriorities.get(5).motor[1].getVelocity() / slidesTickToInch * -1;
            slides1Current = motorPriorities.get(5).motor[0].getCurrent(CurrentUnit.AMPS);
            slides2Current = motorPriorities.get(5).motor[1].getCurrent(CurrentUnit.AMPS);

//            clawTouch = clawLimit.getState();
        } catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "expansion hub failed");
        }
    }

    public double getSlidesLength() { return slidesLength; }

    public double getSlidesVelocity() { return slidesVelocity; }

    public double getTurretHeading() { return turretAngle; }

    public double getTurretVelocity() { return turretVelocity; }

    public int getLeftEncoderPos() { return localizer.encoders[0].currentVal; }

    public int getRightEncoderPos() { return localizer.encoders[1].currentVal; }

    public int getBackEncoderPos() { return localizer.encoders[2].currentVal; }

    public double getLeftEncoderScaleFactor() { return localizer.encoders[0].scaleFactor; }

    public double getRightEncoderScaleFactor() { return localizer.encoders[1].scaleFactor; }

    public double getBackEncoderScaleFactor() { return localizer.encoders[2].scaleFactor; }

    public boolean clawTouched() { return coneInClaw; }

    public double getClawColorReadings() { return clawColorReading; }

    public double getLeftDist() { return leftDist; }

    public double getBatteryVoltage() { return batteryVoltageSensor.getVoltage(); }
}
