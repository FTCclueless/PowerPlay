package org.firstinspires.ftc.teamcode.modules.extension;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.outtake.Outtake;
import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;

public class Extension {
    public double currentExtensionLength = 0.0;
    public double targetExtensionLength = 6.31103;
    public double targetExtensionAngle = 0.0;
    public double extensionPower = 1.0;

    MyServo extension;
    ArrayList<MyServo> servos;

    Outtake outtake;

    public Extension (HardwareMap hardwareMap, ArrayList<MyServo> servos, Outtake outtake) {
        this.servos = servos;
        this.outtake = outtake;

        extension = new MyServo(hardwareMap.servo.get("extension"),"Speed",1,0.0,0.65, 0.65);

        servos.add(0, extension);
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("targetExtensionLength: ", targetExtensionLength);
        TelemetryUtil.packet.put("targetExtensionAngle: ", Math.toDegrees(targetExtensionAngle));
        TelemetryUtil.packet.put("currentExtensionLength: ", currentExtensionLength);
    }

    public void update() {
        updateExtensionValues();
        updateTelemetry();

        extension.setAngle(-targetExtensionAngle, extensionPower);
    }

    public double baseSlidesExtension = 6.31103;
    public double strokeLength = 13.5826845;

    public void setTargetExtensionLength(double length) {
        length = Math.max(baseSlidesExtension, Math.min(length, strokeLength+baseSlidesExtension));
        targetExtensionLength = length;
        outtake.targetExtensionLength = length;
        targetExtensionAngle = Math.acos((targetExtensionLength-(baseSlidesExtension+strokeLength/2))/(-strokeLength/2)); // https://www.desmos.com/calculator/aqezyzoq5y
    }

    public void retractExtension() {
        setTargetExtensionLength(baseSlidesExtension);
    }

    public double getCurrentExtensionLength() {
        return currentExtensionLength;
    }

    public void updateExtensionValues() {
        currentExtensionLength = (baseSlidesExtension + (strokeLength/2)) - (strokeLength/2 * Math.cos(-extension.getAngle()));
    }

    public boolean isInPosition (double length) {
        return Math.abs(targetExtensionLength - currentExtensionLength) <= length;
    }
}
