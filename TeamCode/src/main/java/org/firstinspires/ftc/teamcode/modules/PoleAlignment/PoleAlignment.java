package org.firstinspires.ftc.teamcode.modules.PoleAlignment;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.actuation.Actuation;
import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;

public class PoleAlignment {
    public double currentPoleAlignmentPosition = 0.0;
    public double targetPoleAlignmentPosition = 0.0;
    public double poleAlignmentPower = 1.0;

    MyServo poleAlignment;
    ArrayList<MyServo> servos;
    Actuation actuation;

    double initPosition = 0.682;
    double intakePosition = 0.396; //0.2689
    double depositPosition = 0.149;

    public PoleAlignment(HardwareMap hardwareMap, ArrayList<MyServo> servos, Actuation actuation) {
        this.servos = servos;
        this.actuation = actuation;

        poleAlignment = new MyServo(hardwareMap.servo.get("poleAlignment"),"JX",1,0,1.0, initPosition);

        servos.add(2, poleAlignment);
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("targetPoleAlignmentPosition: ", targetPoleAlignmentPosition);
        TelemetryUtil.packet.put("currentPoleAlignmentPosition: ", currentPoleAlignmentPosition);
    }

    public void update() {
        updatePoleAlignmentValues();
        updateTelemetry();

        poleAlignment.setPosition(targetPoleAlignmentPosition, poleAlignmentPower);
    }

    public void setTargetPoleAlignmentPosition(double position) {
        targetPoleAlignmentPosition = position;
    }

    public double getCurrentPoleAlignmentPosition() {
        return currentPoleAlignmentPosition;
    }

    public void updatePoleAlignmentValues() {
        currentPoleAlignmentPosition = poleAlignment.getCurrentPosition();
    }

    public boolean isInPosition (double position) {
        return Math.abs(targetPoleAlignmentPosition - currentPoleAlignmentPosition) <= position;
    }

    public void init() {
        targetPoleAlignmentPosition = initPosition;
    }

    public void intake() {
        targetPoleAlignmentPosition = intakePosition;
    }

    public void deposit() {
        targetPoleAlignmentPosition = depositPosition;
    }
}
