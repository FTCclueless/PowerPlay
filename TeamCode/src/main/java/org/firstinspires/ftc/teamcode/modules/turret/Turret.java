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
    public enum STATE {PICKUP, LEFT, RIGHT, BACK, BACK_LEFT, BACK_RIGHT, ADJUST}

    public STATE currentState = STATE.PICKUP;
    public PID turretPID = new PID(0.0045, 0.0003,0.0);

    public double currentTurretAngle = 0.0;
    public double currentTurretVelocity = 0.0;
    public double targetTurretAngle = 0.0;
    public double targetTurretVelocity = 0.0;
    public double turretPower = 0.0;
    public static double turretPercentMax = 0.98;

    public Turret(HardwareMap hardwareMap, ArrayList<MotorPriority> motorPriorities) {
        this.motorPriorities = motorPriorities;

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        motorPriorities.add(4, new MotorPriority(turret,4,4));
    }

    public void update() {
        updateTurretValues();

        double turretError = targetTurretAngle - currentTurretAngle;
        targetTurretVelocity = Math.max(Math.min(turretError * (230.4596076657823/5), (230.4596076657823*turretPercentMax)),-230.4596076657823*turretPercentMax);
        turretPower = turretPID.update(targetTurretVelocity - currentTurretVelocity);
        motorPriorities.get(4).setTargetPower(turretPower);

        // ALL TURRET VALUES ASSUME FRONT IS THE INTAKE
        switch (currentState) {
            case PICKUP:
                targetTurretAngle = 0.1;
                break;
            case LEFT:
                targetTurretAngle = -90.0;
                break;
            case RIGHT:
                targetTurretAngle = 90.0;
                break;
            case BACK:
                targetTurretAngle = 0.0;
                break;
            case BACK_LEFT:
                targetTurretAngle = -45.0;
                break;
            case BACK_RIGHT:
                targetTurretAngle = 45.0;
                break;
            case ADJUST:
                break;
        }
    }

    public void updateTurretValues() {
        currentTurretAngle = sensors.getTurretAngle();
        currentTurretVelocity = sensors.getTurretVelocity();
    }

    public void moveToPickup() { currentState = STATE.PICKUP; }
    public void moveToLeft() { currentState = STATE.LEFT; }
    public void moveToRight() { currentState = STATE.RIGHT; }
    public void moveToBack() { currentState = STATE.BACK; }
    public void moveToBackLeft() { currentState = STATE.BACK_LEFT; }
    public void moveToBackRight() { currentState = STATE.BACK_RIGHT; }

    public void moveLeft (double amount) {
        currentState = STATE.ADJUST;
        targetTurretAngle -= amount;
    }

    public void moveRight (double amount) {
        currentState = STATE.ADJUST;
        targetTurretAngle += amount;
    }
}
