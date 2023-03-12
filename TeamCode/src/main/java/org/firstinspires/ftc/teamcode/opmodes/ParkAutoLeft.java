package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Test")
public class ParkAutoLeft extends ParkAuto {
    public ParkAutoLeft() {
        super();
        super.lr = true;
        super.tb = true;
    }
}