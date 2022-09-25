package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.MotorPriority;

import java.util.HashMap;

public class Robot {
    LynxModule controlHub, expansionHub;
    HardwareMap hardwareMap;

    Intake intake;
    Slides slides;
    Sensors sensors;

    HashMap<String, MotorPriority> motorPriorities = new HashMap<String, MotorPriority>();

    public Robot (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        initHubs();

        sensors = new Sensors(motorPriorities, controlHub, expansionHub);

        intake = new Intake(hardwareMap, motorPriorities);
        slides = new Slides(hardwareMap, motorPriorities, sensors);
    }

    public void update() {
        // TODO: update MotorPriorities
        intake.update();
        slides.update();
    }

    public void initHubs() {
        try {
            controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
            controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
            expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } catch (RuntimeException e) {
            throw new RuntimeException("One or more of the REV hubs could not be found. More info: " + e);
        }
    }
}
