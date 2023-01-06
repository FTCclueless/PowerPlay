package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Test")
public class ParkAutoTopRight extends ParkAuto {
    public ParkAutoTopRight() {
        super.lr = false;
        super.tb = true;
    }
}