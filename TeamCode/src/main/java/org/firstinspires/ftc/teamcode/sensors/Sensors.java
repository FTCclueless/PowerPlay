package org.firstinspires.ftc.teamcode.sensors;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MotorPriority;

import java.util.HashMap;

public class Sensors {
    HashMap<String, MotorPriority> motorPriorities;

    public double slidesLength, slidesVelocity;

    public Sensors (HashMap<String, MotorPriority> motorPriorities, LynxModule controlHub, LynxModule expansionHub2) {
        this.motorPriorities = motorPriorities;
    }

    public void updateHub1() {
        try {
            slidesLength = motorPriorities.get("slides").motor[0].getCurrentPosition() / 33.5163131655;
            slidesVelocity = motorPriorities.get("slides").motor[0].getVelocity() / 33.5163131655;
        } catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "control hub failed");
        }
    }

    // TODO: add in updateHub2()

    public double getSlidesLength() {
        return slidesLength;
    }

    public double getSlidesVelocity() {
        return slidesVelocity;
    }
}
