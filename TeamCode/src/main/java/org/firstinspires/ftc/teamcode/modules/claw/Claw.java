package org.firstinspires.ftc.teamcode.modules.claw;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;

public class Claw {
    MyServo claw;

    public double currentClawPosition = 0.0;
    public double targetClawPosition = 0.0;
    public double clawPower = 1.0;

    public double intakePosition = 0.275;
    public double closePosition = 0.0;
    public double openPosition = 0.26499;
    public double fullOpenPosition = 0.275;

    public enum STATE {OPEN, INTAKE, CLOSED, FULL_OPEN}
    public STATE currentState = STATE.OPEN;

    ArrayList<MyServo> servos;

    public Claw(HardwareMap hardwareMap, ArrayList<MyServo> servos) {
        this.servos = servos;

        claw = new MyServo(hardwareMap.servo.get("claw"),"Amazon",1,0,1);

        servos.add(2, claw);
    }

    public void update() {
        updateClawValues();

        switch (currentState) {
            case OPEN:
                setTargetClawPosition(openPosition);
                break;
            case INTAKE:
                setTargetClawPosition(intakePosition);
                break;
            case CLOSED:
                setTargetClawPosition(closePosition);
                break;
            case FULL_OPEN:
                setTargetClawPosition(fullOpenPosition);
                break;
        }

        claw.setPosition(targetClawPosition, clawPower);
    }

    public void setTargetClawPosition(double position) {
        targetClawPosition = position;
    }

    public double getCurrentClawAngle () {
        return currentClawPosition;
    }

    public void updateClawValues() {
        currentClawPosition = claw.getCurrentPosition();
    }

    public void open() {
        currentState = STATE.OPEN;
    }

    public void close() {
        currentState = STATE.CLOSED;
    }

    public void intake() {
        currentState = STATE.INTAKE;
    }

    public void fullOpen() {
        currentState = STATE.FULL_OPEN;
    }

    public boolean isOpen () {
        if (currentState == STATE.OPEN) {
            return true;
        } else {
            return false;
        }
    }

    public void move() {
        if(isOpen()) {
            close();
        } else {
            open();
        }
    }
}
