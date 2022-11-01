package org.firstinspires.ftc.teamcode.modules.intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.MotorPriority;
import org.firstinspires.ftc.teamcode.util.MyServo;

import java.util.ArrayList;

public class ServoIntake {
    public DcMotorEx intake;
    public enum STATE {ON, OFF, REVERSE}

    double intakePower = 1.0;

    CRServo intake1, intake2;
    public STATE currentState = STATE.OFF;

    public ServoIntake(HardwareMap hardwareMap) {
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
    }

    public void update() {
        switch (currentState) {
            case ON:
                intake1.setPower(intakePower);
                intake2.setPower(intakePower);
                break;
            case OFF:
                intake1.setPower(0.0);
                intake2.setPower(0.0);
                break;
            case REVERSE:
                intake1.setPower(-intakePower);
                intake2.setPower(-intakePower);
                break;
        }
    }

    public void on() {
        currentState = STATE.ON;
    }

    public void off() {
        currentState = STATE.OFF;
    }

    public void reverse() {
        currentState = STATE.REVERSE;
    }
}
