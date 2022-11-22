package org.firstinspires.ftc.teamcode.modules.actuation;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MyServo;

import java.util.ArrayList;

public class Actuation {
    public double currentActPosition = 0.0;
    public double targetActPosition = 0.0;
    public double actPower = 0.0;

    MyServo act;
    ArrayList<MyServo> servos;

    double levelPosition = 0.0;
    double tiltedPosition = 0.5;

    public Actuation(HardwareMap hardwareMap, ArrayList<MyServo> servos) {
        this.servos = servos;

        act = new MyServo(hardwareMap.servo.get("act"),"Torque",1,0,1);

        servos.add(1, act);
    }

    public void update() {
        updateActValues();

        act.setPosition(targetActPosition, actPower);
    }

    public void setTargetActPosition(double position) {
        targetActPosition = position;
    }

    public double getCurrentActPosition() {
        return currentActPosition;
    }

    public void updateActValues() {
        currentActPosition = act.getCurrentPosition();
    }

    public boolean isInPosition (double position) {
        return Math.abs(targetActPosition - currentActPosition) <= position;
    }

    public void level() {
        act.setPosition(levelPosition, actPower);
    }

    public void tilted() {
        act.setPosition(tiltedPosition, actPower);
    }
}
