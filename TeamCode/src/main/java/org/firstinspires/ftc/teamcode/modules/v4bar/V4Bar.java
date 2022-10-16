package org.firstinspires.ftc.teamcode.modules.v4bar;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MyServo;

public class V4Bar {
    public double currentV4BarAngle = 0.0;
    public double targetV4BarAngle = 0.0;
    public double v4barPower = 0.0;

    MyServo v4bar1, v4bar2;

    public V4Bar(HardwareMap hardwareMap) {
        v4bar1 = new MyServo(hardwareMap.servo.get("v4bar1"),"Speed",1,0,1);
        v4bar2 = new MyServo(hardwareMap.servo.get("v4bar2"),"Speed",1,0,1);
    }

    public void update() {
        updateV4BarValues();

        v4bar1.setAngle(targetV4BarAngle, v4barPower);
        v4bar2.setAngle(targetV4BarAngle, v4barPower);
    }

    public void setTargetV4BarAngle(double angle) {
        targetV4BarAngle = angle;
    }

    public void setTargetV4BarAngle(double angle, double power) {
        targetV4BarAngle = angle;
        v4barPower = power;
    }

    public double getCurrentV4BarAngle () {
        return currentV4BarAngle;
    }

    public void updateV4BarValues() {
        currentV4BarAngle = v4bar1.getAngle();
    }

    public boolean isInPosition (double angle) {
        if(Math.abs(targetV4BarAngle - currentV4BarAngle) <= Math.toRadians(angle)) {
            return true;
        } else {
            return false;
        }
    }
}
