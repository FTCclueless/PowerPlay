package org.firstinspires.ftc.teamcode.modules.odoLifter;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MyServo;

import java.util.ArrayList;

public class OdoLifter {
    MyServo odoLifter;
    ArrayList<MyServo> servos;

    public double currentOdoLifterPosition = 0.0;
    public double targetOdoLifterPosition = 0.0;
    public double odoLifterPower = 1.0;

    double upPosition = 0.863;
    double downPosition = 0.171;

    public OdoLifter(HardwareMap hardwareMap, ArrayList<MyServo> servos) {
        this.servos = servos;

        odoLifter = new MyServo(hardwareMap.servo.get("odoLifter"),"Torque",1,0,1.0, downPosition);

        servos.add(5, odoLifter);
    }

    public void update() {
        updateOdoLifterValues();
        odoLifter.setPosition(targetOdoLifterPosition, odoLifterPower);
    }

    public void updateOdoLifterValues() {
        currentOdoLifterPosition = odoLifter.getCurrentPosition();
    }

    public void up() {
        targetOdoLifterPosition = upPosition;
    }

    public void down () {
        targetOdoLifterPosition = downPosition;
    }
}
