package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Test")
public class ParkAutoRight extends ParkAuto {
    public ParkAutoRight() {
        super();
        super.lr = false;
        super.tb = true;
    }
}