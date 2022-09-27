package org.firstinspires.ftc.teamcode.modules.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MotorPriority;

import java.util.ArrayList;
import java.util.HashMap;

public class Intake {
    public DcMotorEx intake;
    public enum STATE {ON, OFF, REVERSE}

    ArrayList<MotorPriority> motorPriorities;
    double intakePower = 1.0;

    public STATE currentState = STATE.OFF;

    public Intake(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities) {
        this.motorPriorities = motorPriorities;

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        motorPriorities.add(6, new MotorPriority(intake,3,4));
    }

    public void update() {
        switch (currentState) {
            case ON:
                motorPriorities.get(6).setTargetPower(intakePower);
                break;
            case OFF:
                motorPriorities.get(6).setTargetPower(0.0);
                break;
            case REVERSE:
                motorPriorities.get(6).setTargetPower(-intakePower);
                break;
        }
    }

    public void on() {
        currentState = STATE.ON;
    }

    public void off() {
        currentState = STATE.OFF;
    }

    public void reverse() {
        currentState = STATE.REVERSE;
    }
}
