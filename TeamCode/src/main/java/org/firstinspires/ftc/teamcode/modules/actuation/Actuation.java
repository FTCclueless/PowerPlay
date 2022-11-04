package org.firstinspires.ftc.teamcode.modules.actuation;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MyServo;

import java.util.ArrayList;

public class Actuation {
    public double currentActPosition = 0.0;
    public double targetActPosition = 0.0;
    public double actPower = 0.0;

    MyServo act1, act2;
    ArrayList<MyServo> servos;

    double openPosition = 0.5;
    double closePosition = 0.0;

    public Actuation(HardwareMap hardwareMap, ArrayList<MyServo> servos) {
        this.servos = servos;

        act1 = new MyServo(hardwareMap.servo.get("act1"),"Torque",1,0,1);
        act2 = new MyServo(hardwareMap.servo.get("act2"),"Torque",1,0,1);

        servos.add(3, act1);
        servos.add(4, act2);
    }

    public void update() {
        updateActValues();

        act1.setPosition(targetActPosition, actPower);
        act2.setPosition(-targetActPosition, actPower);
    }

    public void setTargetActPosition(double position) {
        targetActPosition = position;
    }

    public double getCurrentActPosition() {
        return currentActPosition;
    }

    public void updateActValues() {
        currentActPosition = act1.getCurrentPosition();
    }

    public boolean isInPosition (double position) {
        if (Math.abs(targetActPosition - currentActPosition) <= position) {
            return true;
        } else {
            return false;
        }
    }

    public void open() {
        act1.setPosition(openPosition, actPower);
        act2.setPosition(-openPosition, actPower);
    }

    public void close() {
        act1.setPosition(closePosition, actPower);
        act2.setPosition(-closePosition, actPower);
    }
}
