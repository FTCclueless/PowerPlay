package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.modules.slides.Slides;
import org.firstinspires.ftc.teamcode.modules.turret.Turret;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.MotorPriority;

import java.util.ArrayList;
import java.util.HashMap;

public class Robot {
    LynxModule controlHub, expansionHub;
    HardwareMap hardwareMap;

    Drivetrain drivetrain;
    Intake intake;
    Turret turret;
    Slides slides;

    Sensors sensors;

    ArrayList<MotorPriority> motorPriorities = new ArrayList<>();

    public Robot (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        initHubs();

        sensors = new Sensors(motorPriorities, controlHub, expansionHub);

        drivetrain = new Drivetrain(hardwareMap, motorPriorities);
        intake = new Intake(hardwareMap, motorPriorities);
        turret = new Turret(hardwareMap, motorPriorities);
        slides = new Slides(hardwareMap, motorPriorities, sensors);
    }

    public void update() {
        loopStart = System.nanoTime();

        updateMotors();

        intake.update();
        turret.update();
        slides.update();

        sensors.updateHub1();
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

    private long loopStart = System.nanoTime();
    double loopTime = 0.0;
    double targetLoopLength = 0.008; //Sets the target loop time in milli seconds
    double numMotorsUpdated = 0;
    double bestMotorUpdate = 1;

    public void updateMotors() {
        numMotorsUpdated = 0;

        while (bestMotorUpdate > 0 && loopTime <= targetLoopLength) { // updates the motors while still time remaining in the loop
            int bestIndex = 0;
            bestMotorUpdate = motorPriorities.get(0).getPriority(targetLoopLength - loopTime);

            // finds motor that needs updating the most
            for (int i = 1; i < motorPriorities.size(); i++) { //finding the motor that is most in need of being updated;
                double currentMotor = motorPriorities.get(i).getPriority(targetLoopLength - loopTime);
                if (currentMotor > bestMotorUpdate) {
                    bestIndex = i;
                    bestMotorUpdate = currentMotor;
                }
            }
            if (bestMotorUpdate != 0) { // priority # of motor needing update the most
                motorPriorities.get(bestIndex).update(); // Resetting the motor priority so that it knows that it updated the motor and setting the motor of the one that most needs it
                numMotorsUpdated += motorPriorities.get(bestIndex).motor.length; //adds the number of motors updated
            }
            updateLoopTime();
        }
        updateLoopTime();
    }

    public void updateLoopTime(){
        loopTime = (System.nanoTime() - loopStart) / 1000000000.0; // converts from nano secs to secs
    }
}
