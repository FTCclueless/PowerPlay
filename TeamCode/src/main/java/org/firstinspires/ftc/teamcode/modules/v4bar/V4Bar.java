package org.firstinspires.ftc.teamcode.modules.v4bar;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.outtake.Outtake;
import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import java.util.ArrayList;

public class V4Bar {
    public double currentV4BarAngle = 0.0;
    public double targetV4BarAngle = 0.0;
    public double v4barPower = 1.0;

    MyServo v4bar1, v4bar2;
    ArrayList<MyServo> servos;

    Outtake outtake;

    public V4Bar(HardwareMap hardwareMap, ArrayList<MyServo> servos, Outtake outtake) {
        this.servos = servos;
        this.outtake = outtake;

        v4bar1 = new MyServo(hardwareMap.servo.get("v4bar1"),"Amazon",1,0,1, 0.718);
        v4bar2 = new MyServo(hardwareMap.servo.get("v4bar2"),"Amazon",1,0,1,0.18599);

        servos.add(0, v4bar1);
        servos.add(1, v4bar2);
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("targetV4BarAngle: ", Math.toDegrees(targetV4BarAngle));
        TelemetryUtil.packet.put("currentV4BarAngle: ", Math.toDegrees(currentV4BarAngle));
    }

    public void update() {
        updateV4BarValues();

        v4bar1.setAngle(-targetV4BarAngle, v4barPower);
        v4bar2.setAngle(targetV4BarAngle, v4barPower);

        updateTelemetry();
    }

    public void setTargetV4BarAngle(double angle) {
        targetV4BarAngle = angle;
        outtake.targetV4BarAngle = angle;
    }

    public void setTargetV4BarAngle(double angle, double power) {
        targetV4BarAngle = angle;
        v4barPower = power;
    }

    public double getCurrentV4BarAngle () {
        return currentV4BarAngle;
    }

    public void updateV4BarValues() {
        currentV4BarAngle = v4bar2.getAngle();
    }

    public boolean isInPosition (double angle) {
        if(Math.abs(targetV4BarAngle - currentV4BarAngle) <= Math.toRadians(angle)) {
            return true;
        } else {
            return false;
        }
    }
}
