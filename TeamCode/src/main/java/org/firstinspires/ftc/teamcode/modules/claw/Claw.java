package org.firstinspires.ftc.teamcode.modules.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MyServo;

import java.util.ArrayList;

public class Claw {
    MyServo claw;

    public double currentClawPosition = 0.0;
    public double targetClawPosition = 0.0;
    public double clawPower = 0.0;

    double openPosition = 0.6;
    double intakePosition = 0.4;
    double closePosition = 0.05;

    public enum STATE {OPEN, INTAKE, CLOSED}
    public STATE currentState = STATE.OPEN;

    ArrayList<MyServo> servos;

    public Claw(HardwareMap hardwareMap, ArrayList<MyServo> servos) {
        this.servos = servos;

        claw = new MyServo(hardwareMap.servo.get("claw"),"Speed",1,0,1);

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
        }

        claw.setPosition(targetClawPosition, clawPower);
    }

    private void setTargetClawPosition(double position) {
        targetClawPosition = position;
    }

    private void setTargetClawAngle(double angle, double power) {
        targetClawPosition = angle;
        clawPower = power;
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
}
