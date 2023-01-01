package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Test")
public class AutoTopRight extends Auto {
    public AutoTopRight() {
        super.lr = false;
        super.tb = true;
    }
}
