package org.firstinspires.ftc.teamcode.modules.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MyServo;

import java.util.ArrayList;

public class Claw {
    MyServo claw;
    ArrayList<MyServo> servos;

    public double currentClawPosition = 0.0;
    public double targetClawPosition = 0.0;
    public double clawPower = 1.0;

    public double closePosition = 1.0;
    public double openPosition = 0.786;
    public double parkPosition = 0.537;
    public double initPosition = 0.84099;
    public double initClosePosition = 0.93;

    public Claw(HardwareMap hardwareMap, ArrayList<MyServo> servos) {
        this.servos = servos;

        claw = new MyServo(hardwareMap.servo.get("claw"),"Torque",1,0.0,1.0, openPosition);

        servos.add(2, claw);
    }

    public void update() {
        updateClawValues();

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
        targetClawPosition = openPosition;
    }

    public void close() {
        targetClawPosition = closePosition;
    }

    public void park() {
        targetClawPosition = parkPosition;
    }

    public void init() {
        targetClawPosition = initPosition;
    }

    public void initClose() {
        targetClawPosition = initClosePosition;
    }

    public boolean isOpen () {
        return currentClawPosition == openPosition;
    }

    public void move() {
        if(isOpen()) {
            close();
        } else {
            open();
        }
    }
}
