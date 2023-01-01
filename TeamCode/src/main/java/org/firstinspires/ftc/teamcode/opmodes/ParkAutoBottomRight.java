package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Test")
public class ParkAutoBottomRight extends ParkAuto {
    public ParkAutoBottomRight() {
        super.lr = false;
        super.tb = false;
    }
}