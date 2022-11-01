package org.firstinspires.ftc.teamcode.modules.actuation;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MyServo;

import java.util.ArrayList;

public class Actuation {
    public double currentActrAngle = 0.0;
    public double targetActAngle = 0.0;
    public double actPower = 0.0;

    MyServo act1, act2;
    ArrayList<MyServo> servos;

    double openAngle = 30.0;
    double closeAngle = 0.0;

    public Actuation(HardwareMap hardwareMap, ArrayList<MyServo> servos) {
        act1 = new MyServo(hardwareMap.servo.get("act1"),"Torque",1,0,1);
        act2 = new MyServo(hardwareMap.servo.get("act2"),"Torque",1,0,1);

        servos.add(3, act1);
        servos.add(4, act2);
    }

    public void update() {
        updateActValues();

        act1.setAngle(targetActAngle, actPower);
        act2.setAngle(-targetActAngle, actPower);
    }

    public void setTargetActAngle(double angle) {
        targetActAngle = angle;
    }

    public void setTargetV4BarAngle(double angle, double power) {
        targetActAngle = angle;
        actPower = power;
    }

    public double getCurrentActAngle() {
        return currentActrAngle;
    }

    public void updateActValues() {
        currentActrAngle = act1.getAngle();
    }

    public boolean isInPosition (double angle) {
        if(Math.abs(targetActAngle - currentActrAngle) <= Math.toRadians(angle)) {
            return true;
        } else {
            return false;
        }
    }

    public void open() {
        act1.setAngle(openAngle, actPower);
        act2.setAngle(-openAngle, actPower);
    }

    public void close() {
        act1.setAngle(closeAngle, actPower);
        act2.setAngle(-closeAngle, actPower);
    }
}
