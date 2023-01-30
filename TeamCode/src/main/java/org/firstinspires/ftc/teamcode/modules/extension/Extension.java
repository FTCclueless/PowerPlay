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

    public double targetExtensionLength = 20.0;
    public double targetExtensionAngle = 0.0;

    public double extensionPower = 1.0;

    public double servoMountingBack = 1.24; // the distance the extension servo is from 0,0
    public double clawForward = 2.786; // forward from the center of the cone to where the linkage end mount is
    public double actuationTiltDistance = 2.5;
    public double momentOfInertiaConstant = 0.1;

    MyServo extension;
    ArrayList<MyServo> servos;
    Outtake outtake;
    Actuation actuation;

    public Extension (HardwareMap hardwareMap, ArrayList<MyServo> servos, Outtake outtake, Actuation actuation) {
        this.servos = servos;
        this.outtake = outtake;
        this.actuation = actuation;

        extension = new MyServo(hardwareMap.servo.get("extension"),"ProModeler",0.9, 0.113,0.965, 0.965); // base pos is when extension is all the way out
//        extension = new MyServo(hardwareMap.servo.get("extension"),"Amazon",0.7, 0.0559,0.8809, 0.0559);

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

    public double strokeLength = 22.36;

    public double smallLinkage = 11.18;
    public double bigLinkage = 12.4;
    public double targetLinkageLength = 0.0;

    public double baseSlidesExtension = bigLinkage - smallLinkage + servoMountingBack + clawForward + 0.00001;

    public void setTargetExtensionLength(double length) {
        length = Math.max(baseSlidesExtension, Math.min(length, strokeLength+baseSlidesExtension - 0.00001));

        targetExtensionLength = length;
        outtake.targetExtensionLength = length;

        targetLinkageLength = targetExtensionLength - servoMountingBack - clawForward;


//        targetExtensionAngle = Math.acos((targetExtensionLength-(baseSlidesExtension+strokeLength/2))/(-strokeLength/2)); // https://www.desmos.com/calculator/aqezyzoq5y
        targetExtensionAngle = Math.acos((Math.pow(smallLinkage, 2) + Math.pow(targetLinkageLength, 2) - Math.pow(bigLinkage, 2))/(2*smallLinkage*targetLinkageLength));
        Log.e("targetExtensionAngle", targetExtensionAngle + "");
    }

    public void retractExtension() {
        setTargetExtensionLength(baseSlidesExtension);
    }

    public double getCurrentExtensionLength() {
        return currentExtensionLength;
    }

    public void updateExtensionValues() {
//        currentExtensionLength = (baseSlidesExtension + (strokeLength/2)) - (strokeLength/2 * Math.cos(-extension.getAngle()));
        currentExtensionAngle = extension.getAngle();
        currentExtensionLength = (smallLinkage*Math.cos(currentExtensionAngle) + Math.sqrt(Math.pow(bigLinkage, 2) - Math.pow(smallLinkage, 2) * Math.pow(Math.sin(currentExtensionAngle), 2))) + servoMountingBack + clawForward;
    }

    public boolean isInPosition (double length) {
        return Math.abs(targetExtensionLength - currentExtensionLength) <= length;
    }
    public boolean isInPosition (double target, double length) {
        return Math.abs(target - currentExtensionLength) <= length;
    }
}
