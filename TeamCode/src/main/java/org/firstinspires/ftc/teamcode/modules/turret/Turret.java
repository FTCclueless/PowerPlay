package org.firstinspires.ftc.teamcode.modules.turret;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.MotorPriority;
import org.firstinspires.ftc.teamcode.util.PID;

import java.util.ArrayList;

public class Turret {
    public DcMotorEx turret;
    Sensors sensors;

    ArrayList<MotorPriority> motorPriorities;

    public PID turretPID = new PID(0.0045, 0.0003,0.0);

    public double currentTurretAngle = 0.0;
    public double currentTurretVelocity = 0.0;
    public double targetTurretAngle = 0.0;
    public double targetTurretVelocity = 0.0;
    public double turretPower = 0.0;
    public static double turretPercentMax = 0.98;

    public Turret(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities, Sensors sensors) {
        this.motorPriorities = motorPriorities;
        this.sensors = sensors;

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        motorPriorities.add(4, new MotorPriority(turret,4,4));
    }

    public void update() {
        updateTurretValues();

        double turretError = targetTurretAngle - currentTurretAngle;
        targetTurretVelocity = Math.max(Math.min(turretError * (230.4596076657823/5), (230.4596076657823*turretPercentMax)),-230.4596076657823*turretPercentMax);
        turretPower = turretPID.update(targetTurretVelocity - currentTurretVelocity);
        motorPriorities.get(4).setTargetPower(turretPower);
    }

    public void updateTurretValues() {
        currentTurretAngle = clipAngle(sensors.getTurretHeading());
        currentTurretVelocity = sensors.getTurretVelocity();
    }

    public void setTargetTurretAngle(double angle) {
        targetTurretAngle = clipAngle(angle);
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
}
