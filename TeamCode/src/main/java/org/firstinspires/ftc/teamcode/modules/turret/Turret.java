package org.firstinspires.ftc.teamcode.modules.turret;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.extension.Extension;
import org.firstinspires.ftc.teamcode.modules.outtake.Outtake;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.MotorPriority;
import org.firstinspires.ftc.teamcode.util.PID;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;

@Config
public class Turret {
    public DcMotorEx turret;
    Sensors sensors;
    Outtake outtake;

    ArrayList<MotorPriority> motorPriorities;

    public PID turretPID = new PID(0.8, 0.0,0.0);

    public double currentTurretAngle = 0.0;
    public double currentTurretVelocity = 0.0;
    public double targetTurretAngle = 0.0;
    public double targetTurretVelocity = 0.0;
    public double turretPower = 0.0;
    public double turretError = 0.0;
    public static double kstatic = 0.1;
    public static double turretPercentMax = 0.98;

    double maxTurretSpeed = 8.28238063219; // radians per sec

    public Turret(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors, Outtake outtake) {
        this.motorPriorities = motorPriorities;
        this.sensors = sensors;
        this.outtake = outtake;

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorPriorities.add(4, new MotorPriority(turret,4,4));
    }

    public void resetEncoders () {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("targetTurretAngle: ", Math.toDegrees(targetTurretAngle));
        TelemetryUtil.packet.put("currentTurretAngle: ", Math.toDegrees(currentTurretAngle));

        TelemetryUtil.packet.put("targetTurretVelocity: ", targetTurretVelocity);
        TelemetryUtil.packet.put("currentTurretVelocity: ", currentTurretVelocity);

        TelemetryUtil.packet.put("turretPower: ", turretPower);
        TelemetryUtil.packet.put("turretError: ", Math.toDegrees(turretError));

        Log.e("targetTurretAngle", Math.toDegrees(targetTurretAngle) + "");
        Log.e("currentTurretAngle", Math.toDegrees(currentTurretAngle) + "");
    }

    public void update() {
        updateTurretValues();

        turretError = clipAngle(targetTurretAngle - currentTurretAngle);

        // THIS CODE MAKES SURE THE WIRES DON"T TWIST TOO MUCH
        if (Math.abs(currentTurretAngle + turretError) > Math.toRadians(315)) {
            turretError = targetTurretAngle - currentTurretAngle;
        }

        if (Math.abs(turretError) >= Math.toRadians(180)) {
            turretPID.resetIntegral();
        }

        turretPower = turretPID.update(turretError);
        turretPower *= (outtake.extension.currentExtensionLength - outtake.extension.baseSlidesExtension)/outtake.extension.strokeLength * -0.2 + 1;
        turretPower += ((Math.abs(Math.toDegrees(turretError)) > 0.3) ? kstatic : 0) * Math.signum(turretPower);

        motorPriorities.get(4).setTargetPower(-turretPower);

        updateTelemetry();
    }

    public void updateTurretValues() {
        currentTurretAngle = sensors.getTurretHeading();
        currentTurretVelocity = sensors.getTurretVelocity();
    }

    public void setTargetTurretAngle(double angle) {
        targetTurretAngle = angle;
        outtake.targetTurretAngle = targetTurretAngle;
    }

    public double getCurrentTurretAngle() {
        return currentTurretAngle;
    }

    public double clipAngle(double angle) {
        while (angle > Math.PI) {
            angle -= Math.PI * 2.0;
        }
        while (angle < -Math.PI) {
            angle += Math.PI * 2.0;
        }
        return angle;
    }

    public boolean isInPosition (double angle) {
        return Math.abs(clipAngle(targetTurretAngle - currentTurretAngle)) <= Math.toRadians(angle);
    }

    public boolean isInPosition (double angle, double targetTurretAngle) {
        return Math.abs(clipAngle(targetTurretAngle - currentTurretAngle)) <= Math.toRadians(angle);
    }

    public void updateTurretPID (double p, double i, double d) {
        turretPID.p = p;
        turretPID.i = i;
        turretPID.d = d;
    }
}