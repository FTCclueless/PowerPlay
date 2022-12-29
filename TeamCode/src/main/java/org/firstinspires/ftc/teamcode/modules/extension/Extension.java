package org.firstinspires.ftc.teamcode.modules.extension;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.actuation.Actuation;
import org.firstinspires.ftc.teamcode.modules.outtake.Outtake;
import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;

public class Extension {
    public double currentExtensionLength = 0.0;
    public double currentExtensionAngle = 0.0;

    public double targetExtensionLength = 6.31103;
    public double targetExtensionAngle = 0.0;

    public double extensionPower = 1.0;

    public double servoMountingBack = 1.465;
    public double clawForward = 5.003937;

    MyServo extension;
    ArrayList<MyServo> servos;
    Outtake outtake;
    Actuation actuation;

    double actuationTiltDistance = 2.5;

    public Extension (HardwareMap hardwareMap, ArrayList<MyServo> servos, Outtake outtake, Actuation actuation) {
        this.servos = servos;
        this.outtake = outtake;
        this.actuation = actuation;

        extension = new MyServo(hardwareMap.servo.get("extension"),"Super Speed",1, 0.05,0.727, 0.727);

        servos.add(1, extension);
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("targetExtensionLength: ", targetExtensionLength);
        TelemetryUtil.packet.put("targetExtensionAngle: ", Math.toDegrees(targetExtensionAngle));
        TelemetryUtil.packet.put("currentExtensionLength: ", currentExtensionLength);
        TelemetryUtil.packet.put("currentExtensionAngle: ", currentExtensionAngle);
    }

    public void update() {
        updateExtensionValues();
        updateTelemetry();

        extension.setAngle(-targetExtensionAngle, extensionPower);
    }

    public double strokeLength = 13.5826845;

    public double smallLinkage = strokeLength/2;
    public double bigLinkage = 13.4252;
    public double targetLinkageLength = 0.0;

    public double baseSlidesExtension = bigLinkage - smallLinkage - servoMountingBack + clawForward;

    public void setTargetExtensionLength(double length) {
        length = Math.max(baseSlidesExtension, Math.min(length, strokeLength+baseSlidesExtension));
        targetExtensionLength = length;
        outtake.targetExtensionLength = length;

        targetLinkageLength = targetExtensionLength + servoMountingBack - clawForward;

        if (!actuation.isLevel()) {
            targetExtensionLength += actuationTiltDistance;
        }

//        targetExtensionAngle = Math.acos((targetExtensionLength-(baseSlidesExtension+strokeLength/2))/(-strokeLength/2)); // https://www.desmos.com/calculator/aqezyzoq5y
        targetExtensionAngle = Math.acos((Math.pow(smallLinkage, 2) + Math.pow(targetLinkageLength, 2) - Math.pow(bigLinkage, 2))/(2*smallLinkage*targetLinkageLength));
    }

    public void retractExtension() {
        setTargetExtensionLength(baseSlidesExtension);
    }

    public double getCurrentExtensionLength() {
        return currentExtensionLength;
    }

    public void updateExtensionValues() {
//        currentExtensionLength = (baseSlidesExtension + (strokeLength/2)) - (strokeLength/2 * Math.cos(-extension.getAngle()));
        currentExtensionAngle = -extension.getAngle();
        currentExtensionLength = (smallLinkage*Math.cos(currentExtensionAngle) + Math.sqrt(Math.pow(bigLinkage, 2) - Math.pow(smallLinkage, 2) * Math.pow(Math.sin(currentExtensionAngle), 2))) - servoMountingBack + clawForward;
        if (!actuation.isLevel()) {
            currentExtensionLength += actuationTiltDistance;
        }
    }

    public boolean isInPosition (double length) {
        return Math.abs(targetExtensionLength - currentExtensionLength) <= length;
    }
}
