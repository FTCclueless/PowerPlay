package org.firstinspires.ftc.teamcode;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.actuation.Actuation;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.modules.intake.ServoIntake;
import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.modules.claw.Claw;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.modules.outtake.Outtake;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.MotorPriority;

import java.util.ArrayList;

public class Robot {
    LynxModule controlHub, expansionHub;
    HardwareMap hardwareMap;

    public Drivetrain drivetrain;
    public Outtake outtake;
    public Claw claw;

    public Sensors sensors;
    public Vision vision;

    public ArrayList<MotorPriority> motorPriorities = new ArrayList<>(8);
    public ArrayList<MyServo> servos = new ArrayList<>(3);

    public enum STATE { TEST, IDLE, INTAKE_ROLLER, INTAKE_CLAW, WAIT_FOR_START_SCORING, SCORING, ADJUST, DEPOSIT, RETRACT }
    public STATE currentState = STATE.IDLE;

    public Robot (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        initHubs();

        TelemetryUtil.setup();

        drivetrain = new Drivetrain(hardwareMap, motorPriorities);
        sensors = new Sensors(hardwareMap, motorPriorities, drivetrain.localizer);
        claw = new Claw(hardwareMap, servos);
        outtake = new Outtake(hardwareMap, motorPriorities, sensors, servos);
        vision = new Vision();
    }

    boolean startRollerIntake = false;
    boolean startClawIntake = false;
    boolean startScoring = false;

    Pose2d conePose = new Pose2d(0,0);
    double coneHeight = 5.0;

    Pose2d posePose = new Pose2d(0,0);
    double poleHeight = 32.0;

    long timeSinceClawOpen = 0;

    private long loopStart = System.nanoTime();

    public void update() {
        loopStart = System.nanoTime();
        updateSubSystems();

        switch (currentState) {
            case TEST:
                break;
            case IDLE:
                break;
            case RETRACT:
                outtake.setTargetRelative(5,0,3);
                claw.open();
                if (startRollerIntake) {
                    currentState = STATE.INTAKE_ROLLER;
                }
                if (startClawIntake) {
                    currentState = STATE.INTAKE_CLAW;
                }
                break;
            case INTAKE_ROLLER:
                claw.open();
                outtake.setTargetRelative(5,0,3);
                if(sensors.rollerTouch) {
                   currentState = STATE.WAIT_FOR_START_SCORING;
                }
                break;
            case INTAKE_CLAW:
                claw.open();
                outtake.setTargetGlobal(drivetrain.getPoseEstimate(), conePose, coneHeight);
                if(sensors.clawTouch) {
                    currentState = STATE.WAIT_FOR_START_SCORING;
                }
                break;
            case WAIT_FOR_START_SCORING:
                claw.close();
                outtake.setTargetRelative(2,0,7);
                if (startScoring) {
                    currentState = STATE.SCORING;
                }
                break;
            case SCORING:
                outtake.setTargetGlobal(drivetrain.getPoseEstimate(), posePose, poleHeight);
                if (outtake.isInPosition()) {
                    currentState = STATE.ADJUST;
                }
                break;
            case ADJUST:
                // TODO: Implement Vision + Driver adjustments
                vision.on();
                if (vision.readyToDeposit()) {
                    timeSinceClawOpen = System.currentTimeMillis();
                    currentState = STATE.DEPOSIT;
                }
                break;
            case DEPOSIT:
                claw.open();
                if(System.currentTimeMillis() - timeSinceClawOpen >= 500) {
                    currentState = STATE.RETRACT;
                }
                break;
        }

        TelemetryUtil.sendTelemetry();
    }

    public void startRollerIntake () {
        startRollerIntake = true;
    }

    public void startClawIntake () {
        startClawIntake = true;
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

    double targetLoopLength = 0.1; //Sets the target loop time in milli seconds

    public void updateMotors() {
        double numMotorsUpdated = 0;
        double bestMotorUpdate = 1;

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
                numMotorsUpdated += motorPriorities.get(bestIndex).motor.length; // adds the number of motors updated
            }
            updateLoopTime();
        }
        updateLoopTime();
    }

    double loopTime = 0.0;

    public void updateLoopTime(){
        loopTime = (System.nanoTime() - loopStart) / 1000000000.0; // converts from nano secs to secs
    }

    public void updateSubSystems() {
        sensors.updateHub1();
        sensors.updateHub2();

        drivetrain.update();
        outtake.update();
        claw.update();

        updateMotors();
    }

    public void setConePose (Pose2d pose2d) { conePose = pose2d; }
    public void setPolePose (Pose2d pose2d) { posePose = pose2d; }
    public void setConeHeight (double height) { coneHeight = height; }
    public void setPoleHeight (double height) { poleHeight = height; }

    public void testMode () {currentState = STATE.TEST; }

    public void followTrajectory(Trajectory trajectory) {
        drivetrain.followTrajectoryAsync(trajectory);
        while(drivetrain.isBusy()) {
            update();
        }
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        drivetrain.followTrajectorySequenceAsync(trajectorySequence);
        while(drivetrain.isBusy()) {
            update();
        }
    }
}
