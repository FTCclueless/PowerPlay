package org.firstinspires.ftc.teamcode.modules.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;

public class Claw {
    MyServo claw;

    public double currentClawPosition = 0.0;
    public double targetClawPosition = 0.0;
    public double clawPower = 1.0;

    public double closePosition = 0.0;
    public double openPosition = 0.3;
    public double retractOpenPosition = 0.2;
    public double parkPosition = 0.88;
    public double initPosition = 0.224;
    public double initClosePosition = 0.075;

    public enum STATE {OPEN, RETRACT_OPEN, CLOSED, PARK, INIT, INIT_CLOSE}
    public STATE currentState = STATE.OPEN;

    ArrayList<MyServo> servos;

    public Claw(HardwareMap hardwareMap, ArrayList<MyServo> servos) {
        this.servos = servos;

        claw = new MyServo(hardwareMap.servo.get("claw"),"ProModeler",1,0.0,1.0, openPosition);

        servos.add(2, claw);
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("CLAW STATE: ", currentState.toString());
    }

    public void update() {
        updateClawValues();
        updateTelemetry();

        switch (currentState) {
            case OPEN:
                setTargetClawPosition(openPosition);
                break;
            case RETRACT_OPEN:
                setTargetClawPosition(retractOpenPosition);
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
            case INIT_CLOSE:
                setTargetClawPosition(initClosePosition);
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

    public void retractOpen() {
        currentState = STATE.RETRACT_OPEN;
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

    public void initClose() {
        currentState = STATE.INIT_CLOSE;
    }

    public boolean isOpen () {
        return (currentState == STATE.OPEN && isInPosition(1));
    }

    public boolean isClosed () {
        return (currentState == STATE.CLOSED && isInPosition(1));
    }

    public void move() {
        if(isOpen()) {
            close();
        } else {
            open();
        }
    }

    public boolean isInPosition (double position) {
        return Math.abs(targetClawPosition - currentClawPosition) <= position;
    }
}