package org.firstinspires.ftc.teamcode.sensors;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.modules.drive.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.MotorPriority;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Sensors {
    ArrayList<MotorPriority> motorPriorities;
    HardwareMap hardwareMap;
    ThreeWheelLocalizer localizer;

    public DigitalChannel rollerLimit;
    public DigitalChannel clawLimit;

    public double slidesLength, slidesVelocity;
    public double turretAngle, turretVelocity;

    public boolean rollerTouch = false;
    public boolean clawTouch = false;

    private final VoltageSensor batteryVoltageSensor;

    public Sensors (HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, ThreeWheelLocalizer localizer) {
        this.motorPriorities = motorPriorities;
        this.hardwareMap = hardwareMap;
        this.localizer = localizer;

        // setup bulkReads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // init sensors
        rollerLimit = hardwareMap.get(DigitalChannel.class, "rollerLimit");
        clawLimit = hardwareMap.get(DigitalChannel.class, "clawLimit");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void updateHub1() {
        try {
            localizer.encoders[0].update(motorPriorities.get(0).motor[0].getCurrentPosition()); // left
            localizer.encoders[1].update(motorPriorities.get(3).motor[0].getCurrentPosition()); // right
            localizer.encoders[2].update(motorPriorities.get(1).motor[0].getCurrentPosition()); // back
        }
        catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "control hub failed");
        }
    }

    // TODO: add in updateHub2()
    public void updateHub2() {
        try {
            turretAngle = motorPriorities.get(4).motor[0].getCurrentPosition() / 9.45935828877; // degrees of turret
            turretVelocity = motorPriorities.get(4).motor[0].getVelocity() / 9.45935828877;

            slidesLength = motorPriorities.get(5).motor[0].getCurrentPosition() / 33.5163131655; // inches of slides
            slidesVelocity = motorPriorities.get(5).motor[0].getVelocity() / 33.5163131655;

            rollerTouch = rollerLimit.getState(); // limit switch pressed
            clawTouch = clawLimit.getState();
        } catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "control hub failed");
        }
    }

    public double getSlidesLength() { return slidesLength; }

    public double getSlidesVelocity() { return slidesVelocity; }

    public double getTurretHeading() { return turretAngle; }

    public double getTurretVelocity() { return turretVelocity; }

    public boolean rollerTouched() { return rollerTouch; }

    public boolean clawTouched() { return clawTouch; }

    public int getLeftEncoderPos() { return localizer.encoders[0].currentVal; }

    public int getRightEncoderPos() { return localizer.encoders[1].currentVal; }

    public int getBackEncoderPos() { return localizer.encoders[2].currentVal; }

    public double getLeftEncoderScaleFactor() { return localizer.encoders[0].scaleFactor; }

    public double getRightEncoderScaleFactor() { return localizer.encoders[1].scaleFactor; }

    public double getBackEncoderScaleFactor() { return localizer.encoders[2].scaleFactor; }

    public double getBatteryVoltage() { return batteryVoltageSensor.getVoltage(); }
}
