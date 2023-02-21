package org.firstinspires.ftc.teamcode.modules.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MyServo;

import java.util.ArrayList;

public class ConeFlipper {
    MyServo coneFlipper1, coneFlipper2;

    double currentConeFlipper1Position = 0.0;
    double currentConeFlipper2Position = 0.0;
    double targetConeFlipper1Position = 0.0;
    double targetConeFlipper2Position = 0.0;

    double coneFlipperPower = 1.0;

    double upPosition1 = 1.0;
    double downPosition1 = 0.3399;
    double upPosition2 = 0.161;
    double downPosition2 = 0.823;

    ArrayList<MyServo> servos;

    public ConeFlipper(HardwareMap hardwareMap, ArrayList<MyServo> servos) {
        this.servos = servos;

        coneFlipper1 = new MyServo(hardwareMap.servo.get("coneFlipper1"),"Speed",1,0.0,1.0, upPosition1);
        coneFlipper2 = new MyServo(hardwareMap.servo.get("coneFlipper2"),"Speed",1,0.0,1.0, upPosition2);

        servos.add(3, coneFlipper1);
        servos.add(4, coneFlipper2);
    }

    public void update() {
        updateConeFlipperValues();

        coneFlipper1.setPosition(targetConeFlipper1Position, coneFlipperPower);
        coneFlipper2.setPosition(targetConeFlipper2Position, coneFlipperPower);
    }

    public void updateConeFlipperValues() {
        currentConeFlipper1Position = coneFlipper1.getCurrentPosition();
        currentConeFlipper2Position = coneFlipper2.getCurrentPosition();
    }

    public void upLeft () {
        targetConeFlipper1Position = upPosition1;
    }

    public void upRight () {
        targetConeFlipper2Position = upPosition2;
    }

    public void downLeft () {
        targetConeFlipper1Position = downPosition1;
    }

    public void downRight () {
        targetConeFlipper2Position = downPosition2;
    }

    public void retract () {
        upLeft();
        upRight();
    }

    public void extend () {
        downLeft();
        downRight();
    }

}