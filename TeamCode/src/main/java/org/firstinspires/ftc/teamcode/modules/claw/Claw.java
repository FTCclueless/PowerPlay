package org.firstinspires.ftc.teamcode.modules.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MyServo;

import java.util.ArrayList;

public class Claw {
    MyServo claw;

    public double currentClawPosition = 0.0;
    public double targetClawPosition = 0.0;
    public double clawPower = 1.0;

    public double closePosition = 0.0;
    public double openPosition = 0.317;
    public double parkPosition = 0.521;
    public double initPosition = 0.206;

    public enum STATE {OPEN, CLOSED, PARK, INIT}
    public STATE currentState = STATE.OPEN;

    ArrayList<MyServo> servos;

    public Claw(HardwareMap hardwareMap, ArrayList<MyServo> servos) {
        this.servos = servos;

        claw = new MyServo(hardwareMap.servo.get("claw"),"Amazon",1,0.0,1.0, openPosition);

        servos.add(2, claw);
    }

    public void update() {
        updateClawValues();

        switch (currentState) {
            case OPEN:
                setTargetClawPosition(openPosition);
                break;
            case CLOSED:
                setTargetClawPosition(closePosition);
                break;
            case PARK:
                setTargetClawPosition(parkPosition);
                break;
            case INIT:
                setTargetClawPosition(initPosition);
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

    public void park() {
        currentState = STATE.PARK;
    }

    public void init() {
        currentState = STATE.INIT;
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
