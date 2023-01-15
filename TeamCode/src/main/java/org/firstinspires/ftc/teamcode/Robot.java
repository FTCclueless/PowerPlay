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
import org.firstinspires.ftc.teamcode.util.Field;
import org.firstinspires.ftc.teamcode.util.MyServo;
import org.firstinspires.ftc.teamcode.util.Storage;
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

    //TODO: Change variable for field centric adjust
    boolean doFieldCentricAdjust = false;

    public Drivetrain drivetrain;
    public Outtake outtake;
    private Actuation actuation;
    public Claw claw;

    public Sensors sensors;
    public Vision vision;

    public ArrayList<MotorPriority> motorPriorities = new ArrayList<>();
    public ArrayList<MyServo> servos = new ArrayList<>();

    public enum STATE {IDLE, INTAKE_RELATIVE, INTAKE_GLOBAL, WAIT_FOR_START_SCORING, SCORING_GLOBAL, SCORING_RELATIVE, DEPOSIT_TELEOP, DEPOSIT_AUTO, RETRACT }
    public STATE currentState = STATE.IDLE;

    public Robot (HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        initHubs();

        TelemetryUtil.setup();

        drivetrain = new Drivetrain(hardwareMap, motorPriorities);
        sensors = new Sensors(hardwareMap, motorPriorities, drivetrain.localizer);
        outtake = new Outtake(hardwareMap, motorPriorities, sensors, servos);
        actuation = outtake.actuation;
        claw = new Claw(hardwareMap, servos);
        vision = new Vision();
    }

    public boolean isRelative = false;

    public boolean startIntakeRelative = false;
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
    double scoringHeight = 28;
    public int scoringLevel = 3;

    Field field = new Field();

    long timeSinceClawOpen = System.currentTimeMillis();

    private long loopStart = System.nanoTime();

    boolean isAtPoint = false;
    public boolean hasGrabbed = false;

    public boolean isTeleop = false;
    public boolean isAutoAim = false;
    public boolean isWaitForStartScoring180 = false;

    public void update() {
        loopStart = System.nanoTime();
        updateSubSystems();
        updateTelemetry();

        double relativeAngle;
        Pose2d globalArmPos = new Pose2d(0,0);
        Pose2d nearestPole = new Pose2d(0,0);

//        if (isTeleop) {
//            drivetrain.localizer.heading = clipAngle(drivetrain.getExternalHeading() + Storage.autoEndPose.getHeading());;
//        }

        Pose2d drivetrainPoseEstimate = drivetrain.getPoseEstimate();

        switch (currentState) {
            case IDLE:
                break;
            case RETRACT:
                claw.close();
                actuation.level();
                outtake.retract();
                if (startIntakeRelative) {
                    outtake.slides.slidesPercentMax = 1.0;
                    startIntakeRelative = false;
                    claw.open();
                    currentState = STATE.INTAKE_RELATIVE;
                }
                if (startIntakeGlobal) {
                    outtake.slides.slidesPercentMax = 1.0;
                    startIntakeGlobal = false;
                    claw.open();
                    currentState = STATE.INTAKE_GLOBAL;
                }
                break;
            case INTAKE_RELATIVE:
                actuation.level();
                outtake.retract();

                if (outtake.slides.isInPosition(0.25)) {
                    claw.open();
                }

                if (sensors.clawTouch) { // needs an external claw.close()
                    sensors.clawTouch = false;
                    outtake.extension.retractExtension();
                    currentState = STATE.WAIT_FOR_START_SCORING;
                }
                break;
            case INTAKE_GLOBAL:
                if ((Math.abs(drivetrain.getPoseEstimate().getX() - drivePose.getX()) <= 4) && (Math.abs(drivetrain.getPoseEstimate().getY() - drivePose.getY()) <= 4)) {
                    drivePose = drivetrain.getPoseEstimate();
                    isAtPoint = true;
                }

                if ((isAtPoint && (outtake.isInPositionGlobal(drivePose, conePose, 3.5)  && outtake.extension.isInPosition(0.1)) || hasGrabbed)) {
                    hasGrabbed = true;
                    claw.close();
                }
                else {
                    claw.open();
                    startClawCloseTime = System.currentTimeMillis();
                }

                if(sensors.clawTouch || System.currentTimeMillis() - startClawCloseTime > 300) { // needs an external claw.close()
                    claw.close();
                    hasGrabbed = true;
                    outtake.setTargetGlobal(drivePose, conePose, coneHeight + 6);
                    if(outtake.slides.isInPosition(3)) { // needs an external claw.close()
                        isAtPoint = false;
                        hasGrabbed = false;
                        currentState = STATE.SCORING_GLOBAL;
                    }
                }
                else{
                    outtake.setTargetGlobal(drivePose, conePose, coneHeight);
                }
                break;
            case WAIT_FOR_START_SCORING:
                claw.close();
                actuation.level();
                if (isWaitForStartScoring180) {
                    outtake.slides.slidesPercentMax = 1.0;
                    outtake.setTargetRelative(-10,0,10);
                } else {
                    outtake.slides.slidesPercentMax = 1.0;
                    outtake.setTargetRelative(10,0,4);
                }
                if (startScoringRelative) {
                    outtake.slides.slidesPercentMax = 1.0;
                    isWaitForStartScoring180 = false;
                    actuation.tilt();
                    extensionDistance = 12.0;
                    offsetX = 0.0;
                    offsetY = 0.0;
                    angleOffset = 0;
                    startScoringRelative = false;
                    currentState = STATE.SCORING_RELATIVE;
                }

                if (startScoringGlobal) {
                    outtake.slides.slidesPercentMax = 1.0;
                    extensionDistance = 12.0;
                    startScoringGlobal = false;
                    currentState = STATE.SCORING_GLOBAL;
                }
                break;
            case SCORING_RELATIVE:
                outtake.slides.slidesPercentMax = 1.0;

                globalArmPos = outtake.getGlobalArmPose(drivetrainPoseEstimate);
                nearestPole = field.getNearestPole(drivetrainPoseEstimate, globalArmPos, scoringLevel);

                if (isAutoAim) {
                    outtake.setTargetGlobal(drivetrain.getPoseEstimate(), nearestPole, scoringHeight, offsetX, offsetY);
                } else {
                    outtake.setTargetRelative(extensionDistance*Math.cos(Math.toRadians(180) + angleOffset),extensionDistance*Math.sin(Math.toRadians(180) + angleOffset), scoringHeight); // changes dynamically based on driver input
                }

                if (startDeposit) {
                    offsetX = 0.0;
                    offsetY = 0.0;
                    startScoringRelative = false;
                    startDeposit = false;
                    isAutoAim = false;

                    drivetrain.localizer.x += nearestPole.getX() - globalArmPos.getX();
                    drivetrain.localizer.y += nearestPole.getY() - globalArmPos.getY();

                    timeSinceClawOpen = System.currentTimeMillis();
                    currentState = STATE.DEPOSIT_TELEOP;
                }
                break;
            case SCORING_GLOBAL:
                outtake.slides.slidesPercentMax = 1.0;
                globalArmPos = outtake.getGlobalArmPose(drivetrainPoseEstimate);
                nearestPole = field.getNearestPole(drivetrainPoseEstimate, globalArmPos, scoringLevel);

                // checks to see if the drivetrain is near the final scoring pose and if it is then give it it's actual drive pose
                if (Math.abs(drivetrain.getPoseEstimate().getX() - drivePose.getX()) <= 4 && Math.abs(drivetrain.getPoseEstimate().getY() - drivePose.getY()) <= 4) {
                    drivePose = drivetrain.getPoseEstimate();
                    isAtPoint = true;
                }

                if (isAtPoint) {
                    outtake.setTargetGlobal(drivePose, polePose, poleHeight);
                    actuation.tilt();
                }
                else {
                    claw.close();
                    actuation.level();
                    outtake.setTargetGlobal(drivePose, polePose, 12);
                    outtake.extension.retractExtension();
                }

                if (isAtPoint && (outtake.isInPositionGlobal(drivePose, polePose,3.5) && (outtake.extension.isInPosition(0.01)))) {
                    timeSinceClawOpen = System.currentTimeMillis();
                    isAtPoint = false;
                    currentState = STATE.DEPOSIT_AUTO;
                }
                break;
            case DEPOSIT_AUTO:
                outtake.slides.slidesPercentMax = 1.0;
                outtake.setTargetGlobal(drivePose, polePose, poleHeight);

                claw.open();
                if (System.currentTimeMillis() - timeSinceClawOpen >= 300) {
                    actuation.level();
                    currentState = STATE.INTAKE_RELATIVE;
                }
                break;
            case DEPOSIT_TELEOP:
                outtake.slides.slidesPercentMax = 1.0;

                claw.open();
                if (System.currentTimeMillis() - timeSinceClawOpen >= 300) {
                    outtake.slides.setTargetSlidesLength(Math.min(scoringHeight + 6, 32));
                    if ((outtake.slides.isInPosition(2)) || (System.currentTimeMillis() - timeSinceClawOpen >= (700))) {
                        actuation.level();
                        currentState = STATE.INTAKE_RELATIVE;
                    }
                    break;
                }
        }

        drivetrain.trajectorySequenceRunner.globalArmPose = globalArmPos;
        drivetrain.trajectorySequenceRunner.nearestPole = nearestPole;

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

    double extensionDistance = 7.0;

    double offsetX = 0.0;
    double offsetY = 0.0;
    double angleOffset = 0.0;

    double previousScoringPreset = 30;

    public double targetAngle = Math.toRadians(-90);

    public void startScoringRelative(Gamepad gamepad, boolean isBlue, double scoringHeight) {
        if (!startScoringRelative) {
            angleOffset = 0.0;
            this.scoringHeight = scoringHeight;
            this.extensionDistance = 12.0;
        }
        startScoringRelative = true;

        if (scoringHeight != previousScoringPreset) {
            this.scoringHeight = scoringHeight;
            previousScoringPreset = scoringHeight;
        }

        boolean amUpdated = false;

//        double m1 = (isBlue ? 1 : -1);
        double m1 = 1;
        double newAngle = 0;
        if (gamepad.dpad_up) { // forward left
            newAngle = Math.toRadians(-45 * m1); //use 90 - 90 if you want it to work for straight across (current 45)
            amUpdated = true;
        }
        else if (gamepad.dpad_right) { // forward right
            newAngle = Math.toRadians(-135 * m1); // use -90 - 90 if you want it to work for straight across (current 45)
            amUpdated = true;
        }
        else if (gamepad.dpad_down) { // back right
            newAngle = Math.toRadians(135 * m1);
            amUpdated = true;
        }
        else if (gamepad.dpad_left) { // back left
            newAngle = Math.toRadians(45 * m1); //use 90 - 90 if you want it to work for straight across (current 45)
            amUpdated = true;
        }

        if ((targetAngle != newAngle) && amUpdated) {
            targetAngle = newAngle;
            extensionDistance = 12.0;
            angleOffset = 0;
            offsetX = 0;
            offsetY = 0;
            this.scoringHeight = scoringHeight;
        }

        //universal
        this.scoringHeight -= gamepad.right_stick_y * 0.25;
        this.scoringHeight = Math.max(0,Math.min(this.scoringHeight, 39.08666));

        if (doFieldCentricAdjust) {
            //global
            offsetX -= gamepad.left_stick_x * 0.075 * m1;
            offsetY += gamepad.left_stick_y * 0.075 * m1;
        }
        else {
            //relative
            angleOffset -= gamepad.left_stick_x * Math.toRadians(0.8);
            angleOffset -= gamepad.right_stick_x * Math.toRadians(0.8);
            extensionDistance -= gamepad.left_stick_y * 0.18375;
            extensionDistance = Math.max(6.31103, Math.min(this.extensionDistance, 23.67279475));

            // adjustments in auto aim
            double globalAngle = drivetrain.localizer.getPoseEstimate().getHeading() + outtake.turret.getCurrentTurretAngle();
            double relativeX = gamepad.left_stick_y * 0.25 * m1;
            double relativeY = gamepad.left_stick_x * 0.25 * m1;
            offsetX -= relativeX*Math.cos(globalAngle) - relativeY*Math.sin(globalAngle);
            offsetY -= relativeY*Math.cos(globalAngle) + relativeX*Math.sin(globalAngle);
        }
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
            expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
            expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } catch (RuntimeException e) {
            throw new RuntimeException("One or more of the REV hubs could not be found. More info: " + e);
        }
    }

    long slides1, turret, slides2 = System.currentTimeMillis();

    public void initPosition (boolean left) {
        double turnSign = left ? 1 : -1;

        actuation.init();
        outtake.extension.retractExtension();
        claw.open();

        slides1 = System.currentTimeMillis();
        outtake.slides.setTargetSlidesLength(12);

        while ((!outtake.slides.isInPosition(2.5)) || (System.currentTimeMillis() - slides1 >= 1000)) {
            Log.e("stuck1", "");
            update();
        }

        turret = System.currentTimeMillis();
        outtake.turret.setTargetTurretAngle(Math.toRadians(55) * turnSign);

        while ((!outtake.turret.isInPosition(5)) || (System.currentTimeMillis() - turret >= 1000)) {
            Log.e("stuck2", "");
            update();
        }

        outtake.slides.slidesPercentMax = 0.25;
        slides2 = System.currentTimeMillis();
        outtake.slides.setTargetSlidesLength(1.0);

        while ((!outtake.slides.isInPosition(2.5)) || (System.currentTimeMillis() - slides2 >= 1000)) {
            Log.e("stuck3", "");
            update();
            outtake.slides.setTargetSlidesLength(1.0);
        }

        outtake.slides.slidesPercentMax = 1.0;
    }

    double targetLoopLength = 0.015; //Sets the target loop time in milli seconds

    public void updateMotors() {
        double numMotorsUpdated = 0;
        double bestMotorUpdate = 1;

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

    public boolean updateStayInPlacePID = false;
    public Pose2d stayInPlacePose = new Pose2d(0,0,0);

    public void updateSubSystems() {
        sensors.updateHub1();
        sensors.updateHub2();

        drivetrain.update();
        outtake.update();
        claw.update();

        if (updateStayInPlacePID) {
            drivetrain.updatePID(stayInPlacePose);
        }

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
