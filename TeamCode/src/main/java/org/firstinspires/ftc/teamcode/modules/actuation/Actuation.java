package org.firstinspires.ftc.teamcode.modules.actuation;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;

public class Actuation {
    public double currentActPosition = 0.0;
    public double targetActPosition = 0.0;
    public double actPower = 1.0;

    MyServo act;
    ArrayList<MyServo> servos;

    double levelPosition = 0.41499;
    double tiltedPosition = 0.2689;
    double initPosition = 0.033;

    public Actuation(HardwareMap hardwareMap, ArrayList<MyServo> servos) {
        this.servos = servos;

        act = new MyServo(hardwareMap.servo.get("act"),"Amazon",1,0,1.0, levelPosition);

        servos.add(0, act);
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("targetActPosition: ", targetActPosition);
        TelemetryUtil.packet.put("currentActPosition: ", currentActPosition);
    }

    public void update() {
        updateActValues();
        updateTelemetry();

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
        targetActPosition = levelPosition;
    }

    public void tilt() {
        targetActPosition = tiltedPosition;
    }

    public void init() {
        targetActPosition = initPosition;
    }

    public boolean isLevel () {
        return targetActPosition == levelPosition;
    }
}
