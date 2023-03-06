package org.firstinspires.ftc.teamcode.modules.PoleAlignment;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.claw.Claw;
import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;

public class PoleAlignment {
    MyServo poleAlignment;
    ArrayList<MyServo> servos;
    Claw claw;

    public double currentPoleAlignmentPosition = 0.0;
    public double targetPoleAlignmentPosition = 0.0;
    public double poleAlignmentPower = 1.0;

    double upPosition = 0.911;
    double downPosition = 0.2389;
    double initPosition = 0.7219;

    public PoleAlignment(HardwareMap hardwareMap, ArrayList<MyServo> servos, Claw claw) {
        this.servos = servos;
        this.claw = claw;

        poleAlignment = new MyServo(hardwareMap.servo.get("poleAlignment"),"Speed",0.5,0,1.0, upPosition);

        servos.add(5, poleAlignment);
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("targetPoleAlignmentPosition: ", targetPoleAlignmentPosition);
        TelemetryUtil.packet.put("currentPoleAlignmentPosition: ", currentPoleAlignmentPosition);
    }

    public void update() {
        updateActValues();
        updateTelemetry();

        poleAlignment.setPosition(targetPoleAlignmentPosition, poleAlignmentPower);
    }

    public void setTargetPoleAlignmentPosition(double position) {
        targetPoleAlignmentPosition = position;
    }

    public double getCurrentPoleAlignmentPosition() {
        return currentPoleAlignmentPosition;
    }

    public void updateActValues() {
        currentPoleAlignmentPosition = poleAlignment.getCurrentPosition();
    }

    public boolean isInPosition (double position) {
        return Math.abs(targetPoleAlignmentPosition - currentPoleAlignmentPosition) <= position;
    }

    public void up() {
        if (!(currentPoleAlignmentPosition == upPosition)) {
            if (claw.isClosed()) {
                targetPoleAlignmentPosition = upPosition;
            } else {
                claw.close();
            }
        }
    }

    public void down() {
        if (!(currentPoleAlignmentPosition == downPosition)) {
            if (claw.isClosed()) {
                targetPoleAlignmentPosition = downPosition;
            } else {
                claw.close();
            }
        }
    }

    public void forceUp() {
        targetPoleAlignmentPosition = upPosition;
    }

    public void init() {
        targetPoleAlignmentPosition = initPosition;
    }
}
