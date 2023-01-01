package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Test")
public class AutoBottomLeft extends Auto {
    public AutoBottomLeft() {
        super.lr = true;
        super.tb = false;
    }
}