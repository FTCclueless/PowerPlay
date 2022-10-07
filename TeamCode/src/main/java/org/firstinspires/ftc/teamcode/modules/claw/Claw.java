package org.firstinspires.ftc.teamcode.modules.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MyServo;

public class Claw {
    MyServo claw;

    public double currentClawAngle = 0.0;
    public double targetClawAngle = 0.0;
    public double clawPower = 0.0;

    double openAngle = 50.0;
    double intakeAngle = 35.0;
    double closeAngle = 25.0;

    public enum STATE {OPEN, INTAKE, CLOSED}
    public STATE currentState = STATE.OPEN;


    public Claw(HardwareMap hardwareMap) {
        claw = new MyServo(hardwareMap.servo.get("claw"),"Speed",1,0,1);
    }

    public void update() {
        updateClawValues();

        switch (currentState) {
            case OPEN:
                setTargetClawAngle(openAngle);
                break;
            case INTAKE:
                setTargetClawAngle(intakeAngle);
                break;
            case CLOSED:
                setTargetClawAngle(closeAngle);
                break;
        }

        claw.setAngle(targetClawAngle, clawPower);
    }

    private void setTargetClawAngle(double angle) {
        targetClawAngle = angle;
    }

    private void setTargetClawAngle(double angle, double power) {
        targetClawAngle = angle;
        clawPower = power;
    }

    public double getCurrentClawAngle () {
        return currentClawAngle;
    }

    public void updateClawValues() {
        currentClawAngle = claw.getAngle();
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
