package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Test")
public class AutoTopLeft extends Auto {
    public AutoTopLeft() {
        super.lr = true;
        super.tb = true;
    }
}