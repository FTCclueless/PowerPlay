package org.firstinspires.ftc.teamcode.sensors;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MotorPriority;

import java.util.ArrayList;
import java.util.HashMap;

public class Sensors {
    ArrayList<MotorPriority> motorPriorities;

    public double slidesLength, slidesVelocity;
    public double turretAngle, turretVelocity;

    public Sensors (ArrayList<MotorPriority> motorPriorities, LynxModule controlHub, LynxModule expansionHub2) {
        this.motorPriorities = motorPriorities;
    }

    public void updateHub1() {
        try {
            turretAngle = motorPriorities.get(4).motor[0].getCurrentPosition() / 9.45935828877; // degrees of turret
            turretVelocity = motorPriorities.get(4).motor[0].getVelocity() / 9.45935828877;

            slidesLength = motorPriorities.get(5).motor[0].getCurrentPosition() / 33.5163131655;
            slidesVelocity = motorPriorities.get(5).motor[0].getVelocity() / 33.5163131655;
        } catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "control hub failed");
        }
    }

    // TODO: add in updateHub2()
    public void updateHub2() {}

    public double getSlidesLength() {
        return slidesLength;
    }

    public double getSlidesVelocity() {
        return slidesVelocity;
    }

    public double getTurretHeading() {
        return turretAngle;
    }

    public double getTurretVelocity() {
        return turretVelocity;
    }
}
