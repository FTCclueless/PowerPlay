package org.firstinspires.ftc.teamcode.modules.v4bar;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class V4Bar {
    public double currentV4BarAngle = 0.0;
    public double targetV4BarAngle = 0.0;

    public V4Bar(HardwareMap hardwareMap) {}

    public void update() {
        currentV4BarAngle = targetV4BarAngle;
    }

    public void setTargetV4BarAngle(double angle) {
        targetV4BarAngle = angle;
    }

    public double getCurrentV4BarAngle () {
        return currentV4BarAngle;
    }
}
