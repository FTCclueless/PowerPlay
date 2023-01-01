package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Test")
public class ParkAutoTopLeft extends ParkAuto {
    public ParkAutoTopLeft() {
        super.lr = true;
        super.tb = true;
    }
}