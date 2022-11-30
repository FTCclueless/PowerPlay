package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.modules.actuation.Actuation;
import org.firstinspires.ftc.teamcode.modules.drive.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.firstinspires.ftc.teamcode.vision.Vision;
import org.firstinspires.ftc.teamcode.modules.claw.Claw;
import org.firstinspires.ftc.teamcode.modules.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.modules.outtake.Outtake;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.util.MotorPriority;

import java.util.ArrayList;

public class Robot {
    LynxModule controlHub, expansionHub;
    HardwareMap hardwareMap;

    public Drivetrain drivetrain;
    public Outtake outtake;
    public Actuation actuation;
    public Claw claw;

    public Sensors sensors;
    public Vision vision;

    public ArrayList<MotorPriority> motorPriorities = new ArrayList<>();
    public ArrayList<MyServo> servos = new ArrayList<>();

    public enum STATE {IDLE, INIT, INTAKE_RELATIVE, INTAKE_GLOBAL, WAIT_FOR_START_SCORING, WAIT_FOR_START_SCORING_AUTO, SCORING_GLOBAL, SCORING_RELATIVE_WITH_IMU, ADJUST, DEPOSIT, RETRACT }
    public STATE currentState = STATE.INIT;

    public Robot (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        initHubs();

        TelemetryUtil.setup();

        drivetrain = new Drivetrain(hardwareMap, motorPriorities);
        sensors = new Sensors(hardwareMap, motorPriorities, drivetrain.localizer);
        outtake = new Outtake(hardwareMap, motorPriorities, sensors, servos);
        actuation = new Actuation(hardwareMap, servos);
        claw = new Claw(hardwareMap, servos);
        vision = new Vision();
    }

    public boolean isRelative = false;

    boolean startIntakeRelative = false;
    boolean startIntakeGlobal = false;

    boolean startScoringRelative = false;
    boolean startScoringGlobal = false;

    boolean startDeposit = false;

    long startClawCloseTime = System.currentTimeMillis();

    Pose2d conePose = new Pose2d(0,0);
    double coneHeight = 5.0;

    Pose2d polePose = new Pose2d(0,0);
    double poleHeight = 32.0;

    Pose2d drivePose = new Pose2d(0,0);
    double scoringHeight = 0.0;

    long timeSinceClawOpen = 0;

    private long loopStart = System.nanoTime();

    boolean isAtPoint = false;
    boolean hasGrabbed = false;

    public void update() {
        loopStart = System.nanoTime();
        updateSubSystems();
        updateTelemetry();

        double relativeAngle;

        switch (currentState) {
            case IDLE:
                break;
            case INIT:
                actuation.level();
                outtake.retract();
                claw.fullOpen();
                break;
            case RETRACT:
                claw.close();
                actuation.level();
                outtake.retract();
                if (startIntakeRelative) {
                    startIntakeRelative = false;
                    claw.intake();
                    currentState = STATE.INTAKE_RELATIVE;
                }
                if (startIntakeGlobal) {
                    startIntakeGlobal = false;
                    claw.intake();
                    currentState = STATE.INTAKE_GLOBAL;
                }
                break;
            case INTAKE_RELATIVE:
                actuation.level();
                outtake.retract();

                if (sensors.clawTouch) { // needs an external claw.close()
                    sensors.clawTouch = false;
                    currentState = STATE.WAIT_FOR_START_SCORING;
                }
                break;
            case INTAKE_GLOBAL:
                if ((Math.abs(drivetrain.getPoseEstimate().getX() - drivePose.getX()) <= 4) && (Math.abs(drivetrain.getPoseEstimate().getY() - drivePose.getY()) <= 4)) {
                    drivePose = drivetrain.getPoseEstimate();
                    isAtPoint = true;
                }
                outtake.setTargetGlobal(drivePose, conePose, coneHeight);

                // TODO: Add in external claw.close when the outtake global pose is near the cone pose

                Log.e("outtake.isInPosition(): ", outtake.isInPositionGlobal(drivePose, polePose, 1.5) + "");
                Log.e("isAtPoint: ", isAtPoint + "");

                if (isAtPoint && (outtake.isInPositionGlobal(drivePose, conePose, 1.5) || hasGrabbed)) {
                    Log.e("close claw", "");
                    claw.close();
                }
                else {
                    Log.e("intake claw", "");
                    claw.intake();
                    startClawCloseTime = System.currentTimeMillis();
                }

                if(sensors.clawTouch || System.currentTimeMillis() - startClawCloseTime > 300) { // needs an external claw.close()
                    Log.e("here", "");
                    hasGrabbed = true;
                    outtake.slides.setTargetSlidesLength(10);
                    if(sensors.clawTouch || outtake.slides.isInPosition(3)) { // needs an external claw.close()
                        Log.e("here2", "");
                        isAtPoint = false;
                        hasGrabbed = false;
                        currentState = STATE.WAIT_FOR_START_SCORING_AUTO;
                    }
                }
                break;
            case WAIT_FOR_START_SCORING:
                claw.close();
                actuation.level();
                outtake.retract();
                outtake.slides.setTargetSlidesLength(5);
                if (startScoringRelative) {
                    extensionDistance = 7.0;
                    startScoringRelative = false;
                    currentState = STATE.SCORING_RELATIVE_WITH_IMU;
                }
                if (startScoringGlobal) {
                    extensionDistance = 7.0;
                    startScoringGlobal = false;
                    currentState = STATE.SCORING_GLOBAL;
                }
                break;
            case WAIT_FOR_START_SCORING_AUTO:
                claw.close();
                actuation.level();
                outtake.retract();
                outtake.slides.setTargetSlidesLength(15);
                if (startScoringGlobal) {
                    extensionDistance = 7.0;
                    startScoringGlobal = false;
                    currentState = STATE.SCORING_GLOBAL;
                }
                break;
            case SCORING_RELATIVE_WITH_IMU:
                // TODO: make auto manage 0 angle
                double imu = drivetrain.getExternalHeading();
                relativeAngle = clipAngle(targetAngle - clipAngle(imu));

                // field centric offsets (remove the + targetX / targetY)
//                double targetX = offsetX * Math.cos(imu) + offsetY * Math.sin(imu);
//                double targetY = offsetY * Math.cos(imu) - offsetX * Math.sin(imu);

                outtake.setTargetRelative(extensionDistance*Math.cos(relativeAngle),extensionDistance*Math.sin(relativeAngle), this.scoringHeight); // changes dynamically based on driver input

                if (startDeposit) {
//                    offsetX = 0.0;
//                    offsetY = 0.0;
                    startScoringRelative = false;
                    startDeposit = false;
                    timeSinceClawOpen = System.currentTimeMillis();
                    currentState = STATE.DEPOSIT;
                }
                break;
//            case SCORING_RELATIVE_WITHOUT_IMU:
//                outtake.setTargetRelative(extensionDistance*Math.cos(outtake.turret.getCurrentTurretAngle()),extensionDistance*Math.sin(outtake.turret.getCurrentTurretAngle()), this.scoringHeight); // changes dynamically based on driver input
//
//                if (this.scoringHeight > 5) {
//                    currentState = STATE.SCORING_RELATIVE_WITH_IMU;
//                }
//
//                if (startDeposit) {
//                    startScoringRelative = false;
//                    startDeposit = false;
//                    timeSinceClawOpen = System.currentTimeMillis();
//                    currentState = STATE.DEPOSIT;
//                }
//                break;
            case SCORING_GLOBAL:
                // checks to see if the drivetrain is near the final scoring pose and if it is then give it it's actual drive pose
                if (Math.abs(drivetrain.getPoseEstimate().getX() - drivePose.getX()) <= 4 && Math.abs(drivetrain.getPoseEstimate().getY() - drivePose.getY()) <= 4) {
                    drivePose = drivetrain.getPoseEstimate();
                    isAtPoint = true;
                }
                outtake.setTargetGlobal(drivePose, polePose, poleHeight);

                if (isAtPoint && (outtake.isInPositionGlobal(drivePose, polePose, 1.5))) {
                    timeSinceClawOpen = System.currentTimeMillis();
                    isAtPoint = false;
                    currentState = STATE.DEPOSIT;
                }
                break;
            case DEPOSIT:
                claw.open();
                if(System.currentTimeMillis() - timeSinceClawOpen >= 300) {
                    Log.e("Retract Extension", "");
                    outtake.extension.retractExtension();
                    actuation.level();
                    if(System.currentTimeMillis() - timeSinceClawOpen >= 800) {
                        Log.e("Retract Everything", "");
                        currentState = STATE.INTAKE_RELATIVE;
                    }
                }
                break;
        }

        TelemetryUtil.sendTelemetry();
    }

    public void resetEncoders () {
        outtake.resetEncoders();
    }

    public void updateTelemetry () {
        TelemetryUtil.packet.put("Current State: ", currentState);
        TelemetryUtil.packet.put("Loop Time: ", loopTime);
      }


    public void startIntakeRelative() {
        startIntakeRelative = true;
    }

    public void startIntakeGlobal (Pose2d drivePose, Pose2d conePose, double coneHeight) {
        this.drivePose = drivePose;
        this.conePose = conePose;
        this.coneHeight = coneHeight;

        startIntakeGlobal = true;
    }

    enum ScoringDirection {FORWARD, BACKWARD, LEFT, RIGHT, FORWARD_LEFT, FORWARD_RIGHT}
    public ScoringDirection scoringDirection = ScoringDirection.FORWARD;
    double targetAngle = Math.toRadians(-90);
    double extensionDistance = 7.0;
    double lastScoringHeight = 0.0;

//    double offsetX = 0.0;
//    double offsetY = 0.0;

    public void startScoringRelative(Gamepad gamepad, boolean isBlue, double scoringHeight) {
        if (!startScoringRelative) {
            if (isBlue) {
                targetAngle = Math.toRadians(-90);
            } else {
                targetAngle = Math.toRadians(90);
            }
            scoringDirection = ScoringDirection.FORWARD;
            this.scoringHeight = scoringHeight;
            this.extensionDistance = 7.0;
        }

        // this checks if the lastScoringHeight without offset is not equal to the passed in scoring height without offsets, and if it is true then it will set the scoringHeightWithOffset to the one without offset
        if (lastScoringHeight != scoringHeight) {
            this.scoringHeight = scoringHeight;
            lastScoringHeight = scoringHeight;
        }

        if (((gamepad.dpad_up && isBlue) || (gamepad.dpad_down && !isBlue)) && (scoringDirection != ScoringDirection.FORWARD)) {
            scoringDirection = ScoringDirection.FORWARD;
            targetAngle = Math.toRadians(-90);
            extensionDistance = 7.0;
            this.scoringHeight = scoringHeight;
//            offsetX = 0.0;
//            offsetY = 0.0;
        } else if (((gamepad.dpad_down && isBlue) || (gamepad.dpad_up && !isBlue)) && (scoringDirection != ScoringDirection.BACKWARD)) {
            scoringDirection = ScoringDirection.BACKWARD;
            targetAngle = Math.toRadians(90);
            extensionDistance = 7.0;
            this.scoringHeight = scoringHeight;
//            offsetX = 0.0;
//            offsetY = 0.0;
        } else if (((gamepad.dpad_left && isBlue) || (gamepad.dpad_right && !isBlue)) && (scoringDirection != ScoringDirection.LEFT)) {
            scoringDirection = ScoringDirection.LEFT;
            targetAngle = Math.toRadians(0);
            extensionDistance = 7.0;
            this.scoringHeight = scoringHeight;
//            offsetX = 0.0;
//            offsetY = 0.0;
        } else if (((gamepad.dpad_right && isBlue) || (gamepad.dpad_left && !isBlue)) && (scoringDirection != ScoringDirection.RIGHT)) {
            scoringDirection = ScoringDirection.RIGHT;
            targetAngle = Math.toRadians(180);
            extensionDistance = 7.0;
            this.scoringHeight = scoringHeight;
//            offsetX = 0.0;
//            offsetY = 0.0;
        }
//        else if ((gamepad.left_trigger > 0.5) && (scoringDirection != ScoringDirection.FORWARD_RIGHT)) {
//            scoringDirection = ScoringDirection.FORWARD_RIGHT;
//            if (isBlue) {
//                targetAngle = Math.toRadians(-135);
//            } else {
//                targetAngle = Math.toRadians(45);
//            }
//            extensionDistance = 10.0;
//            this.scoringHeight = scoringHeight;
////            offsetX = 0.0;
////            offsetY = 0.0;
//        } else if ((gamepad.left_bumper) && (scoringDirection != ScoringDirection.FORWARD_LEFT)) {
//            scoringDirection = ScoringDirection.FORWARD_LEFT;
//            if (isBlue) {
//                targetAngle = Math.toRadians(-45);
//            } else {
//                targetAngle = Math.toRadians(135);
//            }
//            extensionDistance = 10.0;
//            this.scoringHeight = scoringHeight;
////            offsetX = 0.0;
////            offsetY = 0.0;
//        }

        // Robot Centric Offsets
        targetAngle -= gamepad.left_stick_x * Math.toRadians(0.8);
        targetAngle -= gamepad.right_stick_x * Math.toRadians(0.8);
        extensionDistance -= gamepad.left_stick_y * 0.1225;
        this.scoringHeight -= gamepad.right_stick_y * 0.25; // offsets

        extensionDistance = Math.max(6.31103, Math.min(this.extensionDistance, 19.8937145));
        this.scoringHeight = Math.max(0,Math.min(this.scoringHeight, 39.08666));

        startScoringRelative = true;

        // Field Centric Offsets
//        double allianceMultiplier;
//
//        if (isBlue) {
//            allianceMultiplier = 1;
//        } else {
//            allianceMultiplier = -1;
//        }
//
//        offsetX -= gamepad.left_stick_x * 0.2 * allianceMultiplier;
//        offsetY -= gamepad.left_stick_y * 0.2 * allianceMultiplier;


    }

    public void startScoringGlobal (Pose2d drivePose, Pose2d polePose, double poleHeight) {
        this.drivePose = drivePose;
        this.polePose = polePose;
        this.poleHeight = poleHeight;

        startScoringGlobal = true;
    }

    public void startDepositing () {
        startDeposit = true;
    }

    public void initHubs() {
        try {
            controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
            controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//            expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
//            expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } catch (RuntimeException e) {
            throw new RuntimeException("One or more of the REV hubs could not be found. More info: " + e);
        }
    }

    double targetLoopLength = 0.015; //Sets the target loop time in milli seconds

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
        actuation.update();
        claw.update();

        updateMotors();
    }

    public void setConePose (Pose2d pose2d) { conePose = pose2d; }
    public void setPolePose (Pose2d pose2d) { polePose = pose2d; }
    public void setConeHeight (double height) { coneHeight = height; }
    public void setPoleHeight (double height) { poleHeight = height; }

    public void testMode () {
        currentState = STATE.IDLE;
        resetEncoders();
    }

    public void followTrajectory(Trajectory trajectory, LinearOpMode opMode) {
        drivetrain.followTrajectoryAsync(trajectory);
        while(drivetrain.isBusy() && opMode.opModeIsActive()) {
            update();
        }
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence, LinearOpMode opMode) {
        drivetrain.followTrajectorySequenceAsync(trajectorySequence);
        while(drivetrain.isBusy() && opMode.opModeIsActive()) {
            update();
        }
    }

    public double clipAngle(double angle) {
        while (angle > Math.PI) {
            angle -= Math.PI * 2.0;
        }
        while (angle < -Math.PI) {
            angle += Math.PI * 2.0;
        }
        return angle;
    }
}
