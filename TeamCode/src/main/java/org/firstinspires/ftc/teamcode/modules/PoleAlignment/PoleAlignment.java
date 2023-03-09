package org.firstinspires.ftc.teamcode.modules.PoleAlignment;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.actuation.Actuation;
import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;

public class PoleAlignment {
    MyServo poleAlignment;
    ArrayList<MyServo> servos;
    Actuation actuation;

    public double currentPoleAlignmentPosition = 0.0;
    public double targetPoleAlignmentPosition = 0.0;
    public double poleAlignmentPower = 1.0;

    // pole alignment positions
    double initPosition = 0.703;

    double downLevelPosition = 0.25;
    double undersideRetractLevelPosition = 0.0;
    double oversideRetractLevelPosition = 1.0;
    double oversideRetractTiltPosition = 0.91399;
    double oversideRetractFoldPosition = 1.0;

    double downTiltPosition = 0.28; // 0.3049

    public PoleAlignment(HardwareMap hardwareMap, ArrayList<MyServo> servos, Actuation actuation) {
        this.servos = servos;
        this.actuation = actuation;

        poleAlignment = new MyServo(hardwareMap.servo.get("poleAlignment"),"Speed",0.5,0.0,1.0, oversideRetractLevelPosition);

        servos.add(5, poleAlignment);
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

    public void updatePoleAlignmentValues() {
        currentPoleAlignmentPosition = poleAlignment.getCurrentPosition();
    }

    public boolean isInPosition (double position) {
        return Math.abs(targetPoleAlignmentPosition - currentPoleAlignmentPosition) <= position;
    }

    public boolean isInitPosition() {
        return Math.abs(currentPoleAlignmentPosition - initPosition) <= 0.1;
    }

    public void init() {
        targetPoleAlignmentPosition = initPosition;
    }

    public void down() {
        if (actuation.isLevel()) {
            targetPoleAlignmentPosition = downLevelPosition;
        } else {
            targetPoleAlignmentPosition = downTiltPosition;
        }
    }

    public void undersideRetract() {
        targetPoleAlignmentPosition = undersideRetractLevelPosition;
    }

    public void oversideRetract() {
        if (actuation.isLevel()) {
            targetPoleAlignmentPosition = oversideRetractLevelPosition;
        }
        else if (actuation.isTilted()){
            targetPoleAlignmentPosition = oversideRetractTiltPosition;
        }
        else if (actuation.isFolded()){
            targetPoleAlignmentPosition = oversideRetractFoldPosition;
        }
        else {
            targetPoleAlignmentPosition = oversideRetractTiltPosition;
        }
    }
}
